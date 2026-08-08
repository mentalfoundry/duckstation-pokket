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

PocketStation::PocketStation() = default;

PocketStation::~PocketStation()
{
  if (m_ps)
    psemu_destroy(m_ps);
}

std::string PocketStation::FormatHardwareId(u32 id)
{
  char buf[PSEMU_HARDWARE_ID_STRING_SIZE];
  psemu_format_hardware_id(id, buf, sizeof(buf));
  return std::string(buf);
}

std::string PocketStation::GetDefaultHardwareId(u32 slot)
{
  return FormatHardwareId(slot + 1);
}

std::unique_ptr<PocketStation> PocketStation::Create(std::span<const u8> bios, std::string_view hardware_id,
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

  psemu_reset(ret->m_ps);

  // Before the machine runs: an app reads the serial when it makes a new save, so changing it after
  // boot would not be seen consistently.
  static constexpr u32 FALLBACK_HARDWARE_ID = 1;

  u32 id = FALLBACK_HARDWARE_ID;
  if (!hardware_id.empty())
  {
    const std::string id_str(hardware_id);
    if (!psemu_parse_hardware_id(id_str.c_str(), &id))
    {
      WARNING_LOG("Cannot parse PocketStation hardware ID '{}', using {:08X}.", hardware_id, FALLBACK_HARDWARE_ID);
      id = FALLBACK_HARDWARE_ID;
    }
  }
  psemu_set_hardware_id(ret->m_ps, id);

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

  ret->m_state_size = psemu_state_size(ret->m_ps);

  VERBOSE_LOG("PocketStation booted and docked, state size {} bytes.", ret->m_state_size);
  return ret;
}

bool PocketStation::Transfer(u8 data_in, u8* data_out)
{
  return (psemu_com_transfer(m_ps, data_in, data_out, PSEMU_COM_DEFAULT_TIMEOUT_CYCLES) != 0);
}

void PocketStation::ResetTransferState()
{
  psemu_com_set_selected(m_ps, 0);

  for (u32 i = 0; i < SETTLE_FRAMES; i++)
    psemu_run(m_ps, FRAME_CYCLES);
}

void PocketStation::LoadFlash(const MemoryCardImage::DataArray& data)
{
  if (psemu_load_flash_image(m_ps, data.data(), data.size()) != PSEMU_OK)
    ERROR_LOG("Failed to load card image into PocketStation flash.");
}

bool PocketStation::SaveFlash(MemoryCardImage::DataArray* data) const
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
