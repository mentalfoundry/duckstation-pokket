// SPDX-FileCopyrightText: 2019-2026 Connor McLaughlin <stenzek@gmail.com>
// SPDX-License-Identifier: CC-BY-NC-ND-4.0

#include "pocketstation.h"

#include "util/state_wrapper.h"

#include "common/error.h"
#include "common/file_system.h"
#include "common/log.h"

#include "psemu/psemu.h"

#include <chrono>
#include <cstring>
#include <vector>


LOG_CHANNEL(PocketStation);

namespace {

// The screen of the device refreshes at approximately 32Hz, and psemu_run takes its budget in
// reference cycles. This is one frame of the device.
constexpr u32 FRAME_CYCLES = PSEMU_REFERENCE_CLOCK_HZ / 32;

// Kernel RAM offsets for the active-app slot. The 0x5B/0x5C dispatch helper reads u16[0xD0]
// first (if nonzero) and then u8[0xCE]. See docs/app-notes.md, "App-selection and dispatch".
// psemu_reset leaves RAM at zero. Without a nonzero slot, dispatch goes to slot 0 (the card
// header), which holds no app. All app-function commands then fail with no acknowledge.
constexpr u32 RAM_SLOT_CE = 0x00CEu;
constexpr u32 RAM_SLOT_D0 = 0x00D0u;

// Frames to run before docking, so the BIOS reaches its shell.
constexpr u32 BOOT_FRAMES = 200;

// Frames to wait for the kernel to enable communication after docking. It needs one frame or more,
// because its handler skips the switch-bounce period of a real connector.
constexpr u32 DOCK_FRAMES = 60;

// Reference cycles per chunk. The ARM thread holds m_psemu_mutex for exactly one chunk, then
// releases it so the main thread can call Transfer without waiting more than one chunk duration.
// At maximum ARM clock (8x reference), 256 reference cycles runs in under 20 microseconds on a
// modern host — well within one SIO byte period.
constexpr u32 ARM_IDLE_CHUNK = 256u;

// Extra reference cycles added after a SELECT release, on top of the cycles owed for elapsed
// real time. This gives the BIOS enough time to detect sel_drop_latch and start the app handler
// without waiting for the next periodic tick. 8192 cycles covers the BIOS polling loop at any
// ARM clock setting.
constexpr u64 SELECT_BURST_CYCLES = ARM_IDLE_CHUNK * 32u;

// Frames the BIOS needs after SELECT deasserts to complete a Write Sector flash program.
// bu_test.c measures 30 BIOS-delay frames + 8 settle frames = 38 total for a data sector.
// Directory sector writes are suspected to need more; 50 covers both with margin.
constexpr u32 WRITE_SECTOR_SETTLE_FRAMES = 50u;

// Frames to run after the exit-menu navigation before reading flash.
// choco_exit_probe.c uses 120; 38 is the measured minimum for a single-sector write.
constexpr u32 EXIT_SAVE_SETTLE_FRAMES = 120u;

} // namespace

// Returns the directory slot (1-15) of the first PocketStation app on the card, or 0 when the
// card holds no PocketStation app. A PocketStation app has state 0x51 (first block), the 'P'
// flag at frame offset 0x10, and an MCX0 or MCX1 type marker in its title sector.
static u8 FindFirstAppSlot(const u8* flash_data)
{
  for (u32 frame = 1u; frame <= 15u; frame++)
  {
    if (flash_data[frame * 0x80u] != 0x51u)
      continue;
    if (flash_data[frame * 0x80u + 0x10u] != 'P')
      continue;
    const u32 block_start = frame * 0x2000u;
    if (flash_data[block_start + 0x52u] != 'M' || flash_data[block_start + 0x53u] != 'C' ||
        flash_data[block_start + 0x54u] != 'X')
      continue;
    const u8 rev = flash_data[block_start + 0x55u];
    if (rev != '0' && rev != '1')
      continue;
    return static_cast<u8>(frame);
  }
  return 0u;
}

// The 0x5B/0x5C dispatch helper reads the active-app slot from kernel RAM. After a cold boot
// with no button presses, psemu_reset leaves the slot at zero, which points to the card
// header. All app-function commands then dispatch into invalid code and return no acknowledge.
//
// This function checks the slot the BIOS selected during boot. When it is still zero (no app
// navigated to), it writes the first PocketStation app slot directly into RAM. This has the
// same effect as the user moving to that app in the browse screen: the BIOS then dispatches
// 0x5B/0x5C to the correct function table.
//
// Called lazily — inside the mutex — on the first 0x5B/0x5C command from the PS1. Deferring
// to that point keeps the ARM thread running at slot 0 (BIOS shell, no app) before the PS1
// initiates contact, which prevents the app from auto-initializing its flash save area before
// the PS1 game has a chance to set up the correct state.
static void SetActiveAppSlot(psemu_t* ps)
{
  u8* const ram = psemu_ram_data(ps);
  const u8* const flash = psemu_flash_data(ps);
  if (!ram || !flash)
    return;

  const u16 d0 = static_cast<u16>(ram[RAM_SLOT_D0]) | (static_cast<u16>(ram[RAM_SLOT_D0 + 1u]) << 8u);
  const u8 bios_slot = (d0 != 0u) ? static_cast<u8>(d0) : ram[RAM_SLOT_CE];

  INFO_LOG("PocketStation: RAM CE=0x{:02X} D0=0x{:04X} -> active slot {}.", ram[RAM_SLOT_CE], d0, bios_slot);

  if (bios_slot != 0u)
  {
    // D0 is checked first for 0x5B/0x5C dispatch, but 0x58 reads CE directly. If the BIOS
    // set D0 at boot (app found via directory scan) but left CE at zero (no user navigation),
    // 0x58 dispatches to slot 0 and returns defaults. Mirror bios_slot into CE when CE is
    // unset so both dispatch paths reach the same app.
    if (ram[RAM_SLOT_CE] == 0u)
    {
      ram[RAM_SLOT_CE] = bios_slot;
      INFO_LOG("PocketStation: kernel selected slot {} via D0; mirrored to CE.", bios_slot);
    }
    else
    {
      INFO_LOG("PocketStation: kernel selected app slot {}.", bios_slot);
    }
    return;
  }

  // Dump the card directory so the log shows the card layout at dispatch time.
  for (u32 f = 1u; f <= 15u; f++)
  {
    const u8 state = flash[f * 0x80u];
    const u8 pflag = flash[f * 0x80u + 0x10u];
    const u32 bs = f * 0x2000u;
    const u8 t0 = flash[bs + 0x52u], t1 = flash[bs + 0x53u], t2 = flash[bs + 0x54u], t3 = flash[bs + 0x55u];
    auto safe = [](u8 b) -> char { return (b >= 0x20u && b < 0x7Fu) ? static_cast<char>(b) : '.'; };
    VERBOSE_LOG("PocketStation: dir frame {:2}: state=0x{:02X} P=0x{:02X} type={}{}{}{}", f, state, pflag,
                safe(t0), safe(t1), safe(t2), safe(t3));
  }

  const u8 slot = FindFirstAppSlot(flash);
  if (slot == 0u)
  {
    INFO_LOG("PocketStation: no PocketStation app on card.");
    return;
  }

  ram[RAM_SLOT_CE] = slot;
  INFO_LOG("PocketStation: set active slot to {}.", slot);
}

PocketStation::PocketStation() = default;

PocketStation::~PocketStation()
{
  // PrepareForSave may have already stopped the thread. exchange returns false in that case.
  if (m_arm_running.exchange(false))
  {
    m_wake_cv.notify_one();
    m_arm_thread.join();
  }

  if (m_ps)
    psemu_destroy(m_ps);
}

void PocketStation::PrepareForSave()
{
  // Stop the ARM thread first. The psemu_run calls below must not race it.
  if (m_arm_running.exchange(false))
  {
    m_wake_cv.notify_one();
    m_arm_thread.join();
  }

  if (!m_ps)
    return;

  // Complete any in-flight BIOS flash write. The BIOS programs flash after SELECT deasserts,
  // not during the Write Sector command. The ARM background thread runs those programming delay
  // loops in real time. If the user quits DuckStation within 38 frames of the last Write Sector,
  // the ARM thread stops before the loops finish and the write is lost. Run WRITE_SECTOR_SETTLE_FRAMES
  // synchronously now so any pending write reaches flash before SaveFlash reads the result.
  for (u32 f = 0u; f < WRITE_SECTOR_SETTLE_FRAMES; f++)
    psemu_run(m_ps, FRAME_CYCLES);

  // After the settle, CE may still be zero: the BIOS calls SetActiveAppSlot lazily on the
  // first 0x58 command. If the card was written in this session (e.g. the app was just
  // downloaded), the MCX magic can reach flash after that 0x58 call and leave CE unset.
  // Scanning here gives the exit sequence a valid slot to dispatch against.
  {
    u8* const ram = psemu_ram_data(m_ps);
    const u8* const fl2 = psemu_flash_data(m_ps);
    if (ram && fl2 && ram[RAM_SLOT_CE] == 0u)
    {
      const u8 slot = FindFirstAppSlot(fl2);
      if (slot != 0u)
      {
        ram[RAM_SLOT_CE] = slot;
        INFO_LOG("PocketStation: set active slot to {} (PrepareForSave scan).", slot);
      }
    }
  }

  // Undock so the BIOS returns to standalone mode. The app's game loop must be running
  // before the exit sequence can accumulate a Fire hold.
  psemu_com_set_docked(m_ps, 0);

  // Poll until the app is executing from its FLASH1 window (psemu_app_running), or until
  // the timeout expires. The BIOS needs time after undock to leave the SIO command-wait
  // path and resume the app's standalone game loop. 600 frames is the same budget that
  // choco_exit_probe.c uses for its full cold-boot + navigation sequence; using the same
  // bound here ensures parity with the probe tool's tested conditions.
  u32 app_start_frames = 0u;
  for (; app_start_frames < 600u; app_start_frames++)
  {
    psemu_run(m_ps, FRAME_CYCLES);
    if (psemu_app_running(m_ps))
      break;
  }
  INFO_LOG("PocketStation: app running={} after {} undock frames.", psemu_app_running(m_ps) ? 1 : 0,
           app_start_frames);

  // Trigger the exit-save. Hold Fire for 300 frames: apps that write flash on a sustained
  // press finish here. If flash does not change, the app shows a continue/exit prompt — run
  // the navigation sequence (release → Down → release → Fire → release) to select Exit.
  const u8* const fl = psemu_flash_data(m_ps);
  if (fl && psemu_app_running(m_ps))
  {
    std::vector<u8> snap(fl, fl + PSEMU_FLASH_SIZE);
    psemu_set_buttons(m_ps, PSEMU_BUTTON_FIRE);
    u32 hold_frames = 0u;
    for (; hold_frames < 300u; hold_frames++)
    {
      psemu_run(m_ps, FRAME_CYCLES);
      if (std::memcmp(psemu_flash_data(m_ps), snap.data(), PSEMU_FLASH_SIZE) != 0)
        break;
    }
    psemu_set_buttons(m_ps, 0u);

    if (hold_frames < 300u)
    {
      INFO_LOG("PocketStation: hold-save triggered after {} frames.", hold_frames + 1u);
    }
    else
    {
      // Fire hold alone did not write flash. Navigate the continue/exit prompt:
      // release one frame, then Down to move the cursor to Exit, then Fire to confirm.
      // Sequence matches choco_exit_probe.c: release/Down/release/Fire/release.
      psemu_run(m_ps, FRAME_CYCLES);
      psemu_set_buttons(m_ps, PSEMU_BUTTON_DOWN);
      psemu_run(m_ps, FRAME_CYCLES);
      psemu_set_buttons(m_ps, 0u);
      psemu_run(m_ps, FRAME_CYCLES);
      psemu_set_buttons(m_ps, PSEMU_BUTTON_FIRE);
      psemu_run(m_ps, FRAME_CYCLES);
      psemu_set_buttons(m_ps, 0u);

      for (u32 f = 0u; f < EXIT_SAVE_SETTLE_FRAMES; f++)
        psemu_run(m_ps, FRAME_CYCLES);

      if (std::memcmp(psemu_flash_data(m_ps), snap.data(), PSEMU_FLASH_SIZE) != 0)
        INFO_LOG("PocketStation: exit-save triggered.");
      else
        INFO_LOG("PocketStation: exit-save: no flash change after exit sequence.");
    }
  }
  else if (fl)
  {
    INFO_LOG("PocketStation: app not running after undock; skipping exit sequence.");
  }

  // The dock interrupt handler sets ROT. The undock transition does not fire the handler:
  // INT_IOP is falling, so the interrupt controller clears HOLD rather than setting it, and
  // the kernel ISR never runs. Clear ROT directly so the exported state loads with the
  // standalone orientation in pokketstation.
  psemu_lcd_clear_rot(m_ps);
}

std::unique_ptr<PocketStation> PocketStation::Create(std::span<const u8> bios,
                                                     const MemoryCardImage::DataArray& flash, u32 slot,
                                                     Error* error)
{
  std::unique_ptr<PocketStation> ret(new PocketStation());

  ret->m_ps = psemu_create();
  if (!ret->m_ps)
  {
    Error::SetStringView(error, "Failed to allocate PocketStation machine.");
    return nullptr;
  }

  if (psemu_load_bios(ret->m_ps, bios.data(), bios.size()) != PSEMU_OK)
  {
    Error::SetStringFmt(error, "PocketStation BIOS must be {} bytes, got {}.", static_cast<u32>(PSEMU_BIOS_SIZE),
                        bios.size());
    return nullptr;
  }

  // The BIOS scans the flash directory during boot to set the auto-start slot (RAM[0xCE]).
  // The flash must hold the card data before the boot frames run so that scan finds any app.
  if (psemu_load_flash_image(ret->m_ps, flash.data(), flash.size()) != PSEMU_OK)
  {
    Error::SetStringView(error, "Failed to load flash image into PocketStation.");
    return nullptr;
  }

  ret->m_slot       = slot;
  ret->m_state_size = psemu_state_size(ret->m_ps);

  psemu_reset(ret->m_ps);
  // Before the machine runs: an app reads the serial when it makes a new save, so changing it
  // after boot would not be seen consistently.
  psemu_set_hardware_id(ret->m_ps, slot + 1);
  for (u32 i = 0; i < BOOT_FRAMES; i++)
    psemu_run(ret->m_ps, FRAME_CYCLES);

  // The kernel enables communication from an interrupt handler, and that handler waits before it
  // reads the docking level again. So the condition arrives a frame or more after the call, and a
  // transfer before it would get no answer.
  psemu_com_set_docked(ret->m_ps, 1);

  bool enabled = false;
  for (u32 i = 0; i < DOCK_FRAMES; i++)
  {
    psemu_run(ret->m_ps, FRAME_CYCLES);
    if (psemu_com_is_enabled(ret->m_ps))
    {
      enabled = true;
      break;
    }
  }

  if (!enabled)
  {
    Error::SetStringView(error, "PocketStation BIOS did not enable communication after docking.");
    return nullptr;
  }

  VERBOSE_LOG("PocketStation booted and docked, state size {} bytes.", ret->m_state_size);

  // All synchronous initialization is complete. Start the background ARM thread. From this point
  // on, all psemu calls on the main thread must hold m_psemu_mutex.
  ret->m_arm_running.store(true, std::memory_order_release);
  ret->m_arm_thread = std::thread(&PocketStation::ARMThreadFunc, ret.get());

  return ret;
}

void PocketStation::ARMThreadFunc()
{
  using Clock = std::chrono::steady_clock;
  using Micros = std::chrono::microseconds;

  // Target period: one ARM display frame (32 Hz). The thread sleeps until this period expires or
  // until ResetTransferState signals a SELECT release, whichever comes first.
  static constexpr u64 FRAME_US = 1'000'000u / 32u; // 31250 µs

  auto last = Clock::now();
  u64 carry = 0u; // sub-cycle remainder carried between iterations

  // Flash write probe: snapshot taken once, compared after every tick.
  std::vector<u8> flash_snap(PSEMU_FLASH_SIZE, 0xFFu);
  {
    std::lock_guard<std::mutex> lock(m_psemu_mutex);
    const u8* const fl = psemu_flash_data(m_ps);
    if (fl)
      std::copy(fl, fl + PSEMU_FLASH_SIZE, flash_snap.begin());
  }

  while (m_arm_running.load(std::memory_order_relaxed))
  {
    // Sleep until one ARM frame has elapsed or a SELECT release wakes this thread early.
    std::cv_status wakeup;
    {
      std::unique_lock<std::mutex> lk(m_wake_mutex);
      wakeup = m_wake_cv.wait_for(lk, Micros(FRAME_US));
    }

    if (!m_arm_running.load(std::memory_order_relaxed))
      break;

    // Compute how many reference cycles elapsed real time has earned. The carry accumulates the
    // fractional part so the ARM does not drift from real-time speed over many iterations.
    const auto now = Clock::now();
    const u64 elapsed_us =
      static_cast<u64>(std::max(std::chrono::duration_cast<Micros>(now - last).count(), INT64_C(0)));
    last = now;

    u64 cycles = (elapsed_us * PSEMU_REFERENCE_CLOCK_HZ + carry) / 1'000'000u;
    carry = (elapsed_us * PSEMU_REFERENCE_CLOCK_HZ + carry) % 1'000'000u;

    // After a SELECT release, add extra cycles so the BIOS detects sel_drop_latch and begins
    // the app handler immediately rather than at the next periodic tick.
    if (wakeup == std::cv_status::no_timeout)
      cycles += SELECT_BURST_CYCLES;

    // Cap catch-up to 4 ARM frames. Without this, a long emulator pause (e.g. save-state) would
    // produce a huge debt that holds the mutex for hundreds of milliseconds on the next wake.
    cycles = std::min(cycles, static_cast<u64>(FRAME_CYCLES * 4u));

    while (cycles > 0u && m_arm_running.load(std::memory_order_relaxed))
    {
      const u32 chunk = static_cast<u32>(std::min(cycles, static_cast<u64>(ARM_IDLE_CHUNK)));
      {
        std::lock_guard<std::mutex> lock(m_psemu_mutex);
        psemu_run(m_ps, chunk);
      }
      cycles -= chunk;
    }

    // Detect flash writes that happened during this tick.
    {
      std::lock_guard<std::mutex> lock(m_psemu_mutex);
      const u8* const fl = psemu_flash_data(m_ps);
      if (fl)
      {
        for (u32 i = 0u; i < PSEMU_FLASH_SIZE; )
        {
          if (fl[i] != flash_snap[i])
          {
            u32 end = i + 1u;
            while (end < PSEMU_FLASH_SIZE && fl[end] != flash_snap[end])
              end++;
            INFO_LOG("PocketStation: flash write 0x{:05X}+{} bytes (wakeup={})",
                     i, end - i, wakeup == std::cv_status::no_timeout ? "SELECT" : "timer");
            std::copy(fl + i, fl + end, flash_snap.begin() + i);
            i = end;
          }
          else
          {
            i++;
          }
        }
      }
    }
  }
}

bool PocketStation::Transfer(u8 data_in, u8* data_out)
{
  if (m_cmd_byte_pos == 0u)
    INFO_LOG("PocketStation: sel=0x{:02X}", data_in);
  else if (m_cmd_byte_pos == 1u)
    m_cmd_in_progress = data_in;

  bool acked;
  {
    std::lock_guard<std::mutex> lock(m_psemu_mutex);

    // On the first PocketStation command from the PS1, set the active app slot. All commands from
    // 0x58 upward (0x58 get-function-count, 0x5B/0x5C execute) dispatch via RAM[0xCE], so the slot
    // must be set before the first 0x58 or the BIOS returns slot-0 (card-header) defaults and FF8
    // never transitions from its 0x58-poll loop to the 0x5B/0x5C data exchange.
    // Deferring to this point (rather than Create()) keeps the ARM at slot 0 until the PS1 makes
    // contact, so the app cannot write flash before the PS1 game sets up the correct state.
    // Retry every command until SetActiveAppSlot succeeds. The BIOS may not have set D0 yet
    // when the first 0x58 arrives (the ARM background thread is still in its boot scan), so
    // we keep trying until CE becomes non-zero. Once CE is set the slot is stable.
    if (!m_app_slot_set && m_cmd_byte_pos == 1u && data_in >= 0x58u)
    {
      SetActiveAppSlot(m_ps);
      const u8* const ram = psemu_ram_data(m_ps);
      if (ram && ram[RAM_SLOT_CE] != 0u)
        m_app_slot_set = true;
    }

    if (psemu_cpu_faulted(m_ps))
      WARNING_LOG("PocketStation: CPU faulted before byte {}.", m_cmd_byte_pos);
    // The BIOS FIQ handles each byte with SELECT asserted, runs its phase-2 callback (which
    // copies data and may write flash), then enters a SELECT-drop wait loop. ResetTransferState
    // drops SELECT via psemu_com_set_selected after the PS1 releases /SEL, which is what exits
    // that wait and triggers the end-of-command cleanup. Using psemu_com_transfer here (SELECT
    // stays asserted) matches what the PS1 does and what mock_ps1_dispatch does in the tests.
    acked = (psemu_com_transfer(m_ps, data_in, data_out, PSEMU_COM_DEFAULT_TIMEOUT_CYCLES) != 0);
  }

  if (m_cmd_byte_pos == 1u)
  {
    INFO_LOG("PocketStation: cmd=0x{:02X} flag=0x{:02X} ACK={}", data_in, *data_out, acked);
    if (data_in >= 0x58u)
    {
      const u8* const ram = psemu_ram_data(m_ps);
      if (ram)
      {
        const u16 d0 = static_cast<u16>(ram[RAM_SLOT_D0]) | (static_cast<u16>(ram[RAM_SLOT_D0 + 1u]) << 8u);
        VERBOSE_LOG("PocketStation: 0x{:02X} dispatch: CE=0x{:02X} D0=0x{:04X}.", data_in, ram[RAM_SLOT_CE], d0);
      }
    }
  }
  else if (m_cmd_byte_pos >= 2u && (m_cmd_in_progress == 0x58u || m_cmd_in_progress == 0x5Au ||
                                    m_cmd_in_progress == 0x59u || m_cmd_in_progress == 0x5Du ||
                                    m_cmd_in_progress == 0x5Bu || m_cmd_in_progress == 0x5Cu))
  {
    VERBOSE_LOG("PocketStation: 0x{:02X} byte {} in=0x{:02X} out=0x{:02X} ACK={}", m_cmd_in_progress,
                m_cmd_byte_pos, data_in, *data_out, acked);
  }
  else if (m_cmd_byte_pos >= 2u && m_cmd_byte_pos <= 5u && m_cmd_in_progress == 0x57u)
    VERBOSE_LOG("PocketStation: 0x57 addr byte {} in=0x{:02X} out=0x{:02X} ACK={}", m_cmd_byte_pos, data_in,
                *data_out, acked);
  else if (m_cmd_byte_pos >= 135u && m_cmd_in_progress == 0x57u)
    VERBOSE_LOG("PocketStation: 0x57 tail byte {} in=0x{:02X} out=0x{:02X} ACK={}", m_cmd_byte_pos, data_in,
                *data_out, acked);
  else if (!acked && m_cmd_byte_pos > 1u)
    VERBOSE_LOG("PocketStation: NACK at byte {} of current command (out=0x{:02X}).", m_cmd_byte_pos, *data_out);

  m_cmd_byte_pos++;
  if (!acked)
  {
    m_cmd_byte_pos = 0u;
    m_cmd_in_progress = 0u;
  }
  return acked;
}

void PocketStation::ResetTransferState(bool was_accessed)
{
  m_cmd_byte_pos = 0u;
  m_cmd_in_progress = 0u;

  if (was_accessed)
  {
    {
      std::lock_guard<std::mutex> lock(m_psemu_mutex);
      if (psemu_cpu_faulted(m_ps))
        ERROR_LOG("PocketStation: CPU fault at command end. Register state is invalid.");
      psemu_com_set_selected(m_ps, 0);
    }
    // Wake the ARM thread so the BIOS end-of-command path runs without waiting for the next
    // periodic tick.
    m_wake_cv.notify_one();
  }
}

void PocketStation::LoadFlash(const MemoryCardImage::DataArray& data)
{
  std::lock_guard<std::mutex> lock(m_psemu_mutex);
  if (psemu_load_flash_image(m_ps, data.data(), data.size()) != PSEMU_OK)
    ERROR_LOG("Failed to load card image into PocketStation flash.");
}

bool PocketStation::SaveFlash(MemoryCardImage::DataArray* data)
{
  MemoryCardImage::DataArray flash;
  {
    std::lock_guard<std::mutex> lock(m_psemu_mutex);
    if (psemu_save_flash_image(m_ps, flash.data(), flash.size()) != PSEMU_OK)
    {
      ERROR_LOG("Failed to read PocketStation flash.");
      return false;
    }

    // No slot patch here: the lazy init in Transfer() sets RAM[0xCE] on the first 0x5B/0x5C
    // command, whether the app was present at boot or appeared during a mid-session download.
  }

  if (flash == *data)
    return false;

  *data = flash;
  return true;
}

bool PocketStation::ReadFramebuffer(std::array<u8, PSEMU_LCD_WIDTH * PSEMU_LCD_HEIGHT / 8>& buf)
{
  std::lock_guard<std::mutex> lock(m_psemu_mutex);
  if (!psemu_framebuffer_dirty(m_ps))
    return false;
  const u8* const fb = psemu_get_framebuffer(m_ps);
  std::copy(fb, fb + buf.size(), buf.begin());
  return true;
}


bool PocketStation::DoState(StateWrapper& sw)
{
  // Hold the mutex for the full state operation so the ARM thread does not run concurrently with
  // the serialization.
  std::lock_guard<std::mutex> lock(m_psemu_mutex);

  // The size is the same for every state of the machine, so this is one fixed block.
  const size_t size = m_state_size;

  if (sw.IsReading())
  {
    std::vector<u8> buf(size);
    sw.DoBytes(buf.data(), size);
    if (sw.HasError())
      return false;

    if (psemu_load_state(m_ps, buf.data(), size) != PSEMU_OK)
    {
      ERROR_LOG("Failed to load PocketStation state.");
      return false;
    }
  }
  else
  {
    std::vector<u8> buf(size);
    if (psemu_save_state(m_ps, buf.data(), size) != PSEMU_OK)
    {
      ERROR_LOG("Failed to save PocketStation state.");
      return false;
    }

    sw.DoBytes(buf.data(), size);
  }

  return !sw.HasError();
}
