// SPDX-FileCopyrightText: 2019-2026 Connor McLaughlin <stenzek@gmail.com>
// SPDX-License-Identifier: CC-BY-NC-ND-4.0

#include "pocketstation.h"

#include "util/state_wrapper.h"

#include "common/error.h"
#include "common/log.h"

#include "psemu/psemu.h"

#include <cstring>

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

// Frames to run after releasing the select line, so the kernel can leave its end-of-command wait
// and re-arm the port.
//
// TUNING POINT. This runs on every command boundary, and each frame is real emulation work on the
// ARM core. Too small and the kernel does not finish, so the next command gets no answer. There is
// no way to ask the device whether it is ready, so this is a budget rather than a condition.
constexpr u32 SETTLE_FRAMES = 4;

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
static void SetActiveAppSlot(psemu_t* ps, const MemoryCardImage::DataArray& flash)
{
  u8* const ram = psemu_ram_data(ps);
  if (!ram)
    return;

  const u16 d0 = static_cast<u16>(ram[RAM_SLOT_D0]) | (static_cast<u16>(ram[RAM_SLOT_D0 + 1u]) << 8u);
  const u8 bios_slot = (d0 != 0u) ? static_cast<u8>(d0) : ram[RAM_SLOT_CE];

  // Log RAM state so the log shows whether boot set a slot or left it at zero.
  INFO_LOG("PocketStation: RAM CE=0x{:02X} D0=0x{:04X} -> active slot {}.", ram[RAM_SLOT_CE], d0, bios_slot);

  if (bios_slot != 0u)
  {
    INFO_LOG("PocketStation: kernel selected app slot {}.", bios_slot);
    return;
  }

  // Dump the card directory so the log shows the card layout.
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

  const u8 slot = FindFirstAppSlot(flash.data());
  if (slot == 0u)
  {
    INFO_LOG("PocketStation: no PocketStation app on card.");
    return;
  }

  // Compare card data with psemu_flash_data to show whether BIOS modified flash during boot.
  const u8* const ps_flash = psemu_flash_data(ps);
  if (ps_flash)
  {
    const u32 bs = slot * 0x2000u;
    auto safe = [](u8 b) -> char { return (b >= 0x20u && b < 0x7Fu) ? static_cast<char>(b) : '.'; };
    INFO_LOG("PocketStation: slot {} card type={}{}{}{} psemu type={}{}{}{}.",
             slot,
             safe(flash[bs + 0x52u]), safe(flash[bs + 0x53u]), safe(flash[bs + 0x54u]), safe(flash[bs + 0x55u]),
             safe(ps_flash[bs + 0x52u]), safe(ps_flash[bs + 0x53u]), safe(ps_flash[bs + 0x54u]),
             safe(ps_flash[bs + 0x55u]));
  }

  ram[RAM_SLOT_CE] = slot;
  INFO_LOG("PocketStation: kernel selected no app; set active slot to {}.", slot);
}

PocketStation::PocketStation() = default;

PocketStation::~PocketStation()
{
  if (m_ps)
    psemu_destroy(m_ps);
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

  psemu_reset(ret->m_ps);

  ret->m_slot = slot;

  // Before the machine runs: an app reads the serial when it makes a new save, so changing it after
  // boot would not be seen consistently.
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

  SetActiveAppSlot(ret->m_ps, flash);

  ret->m_state_size = psemu_state_size(ret->m_ps);

  VERBOSE_LOG("PocketStation booted and docked, state size {} bytes.", ret->m_state_size);
  return ret;
}

bool PocketStation::Transfer(u8 data_in, u8* data_out)
{
  if (psemu_cpu_faulted(m_ps))
    WARNING_LOG("PocketStation: CPU faulted before byte {}.", m_cmd_byte_pos);
  const bool acked = (psemu_com_transfer(m_ps, data_in, data_out, PSEMU_COM_DEFAULT_TIMEOUT_CYCLES) != 0);
  // Byte 0 is the PS1 device-address byte (0x81). Byte 1 is the command byte.
  if (m_cmd_byte_pos == 0u)
    INFO_LOG("PocketStation: sel=0x{:02X} out=0x{:02X} ACK={}", data_in, *data_out, acked);
  else if (m_cmd_byte_pos == 1u)
  {
    m_cmd_in_progress = data_in;
    INFO_LOG("PocketStation: cmd=0x{:02X} flag=0x{:02X} ACK={}", data_in, *data_out, acked);
  }
  else if (m_cmd_byte_pos >= 2u && (m_cmd_in_progress == 0x58u || m_cmd_in_progress == 0x5Du))
    INFO_LOG("PocketStation: 0x{:02X} byte {} in=0x{:02X} out=0x{:02X} ACK={}", m_cmd_in_progress,
             m_cmd_byte_pos, data_in, *data_out, acked);
  else if (m_cmd_byte_pos >= 135u && m_cmd_in_progress == 0x57u)
    INFO_LOG("PocketStation: 0x57 tail byte {} in=0x{:02X} out=0x{:02X} ACK={}", m_cmd_byte_pos, data_in,
             *data_out, acked);
  else if (!acked && m_cmd_byte_pos > 1u)
    INFO_LOG("PocketStation: NACK at byte {} of current command (out=0x{:02X}).", m_cmd_byte_pos, *data_out);
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
  psemu_com_set_selected(m_ps, 0);

  if (was_accessed)
  {
    if (psemu_cpu_faulted(m_ps))
      ERROR_LOG("PocketStation: CPU fault at command end. Register state is invalid.");
    for (u32 i = 0; i < SETTLE_FRAMES; i++)
      psemu_run(m_ps, FRAME_CYCLES);
  }
}

void PocketStation::LoadFlash(const MemoryCardImage::DataArray& data)
{
  if (psemu_load_flash_image(m_ps, data.data(), data.size()) != PSEMU_OK)
    ERROR_LOG("Failed to load card image into PocketStation flash.");
}

bool PocketStation::SaveFlash(MemoryCardImage::DataArray* data)
{
  MemoryCardImage::DataArray flash;
  if (psemu_save_flash_image(m_ps, flash.data(), flash.size()) != PSEMU_OK)
  {
    ERROR_LOG("Failed to read PocketStation flash.");
    return false;
  }

  // The device writes its own flash while an app runs, so a change can happen with no console write
  // at all. That is why this compares instead of trusting a write path to have set a flag.
  if (flash == *data)
    return false;

  // When an app first appears in flash (valid directory entry now but absent before), patch
  // RAM[0xCE] so the kernel dispatches 0x5B/0x5C to the correct function table. This covers
  // the case where the card was blank at boot and an app was downloaded mid-session.
  if (FindFirstAppSlot(data->data()) == 0u && FindFirstAppSlot(flash.data()) != 0u)
    SetActiveAppSlot(m_ps, flash);

  *data = flash;
  return true;
}

bool PocketStation::DoState(StateWrapper& sw)
{
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
