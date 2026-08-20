// SPDX-FileCopyrightText: 2019-2026 Connor McLaughlin <stenzek@gmail.com>
// SPDX-License-Identifier: CC-BY-NC-ND-4.0

#pragma once

#include "memory_card_image.h"
#include "psemu/psemu.h"

#include "common/types.h"

#include <array>
#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <span>
#include <thread>

class Error;
class StateWrapper;

struct psemu;

// A PocketStation in a memory card slot: a memory card with an ARM7 machine inside it.
//
// This class implements no memory card commands, not even the standard ones. The device's BIOS
// answers them, so this moves bytes and runs the machine.
//
// Requires a PocketStation BIOS image; without one nothing answers a transfer.
class PocketStation
{
public:
  PocketStation();
  ~PocketStation();

  // Boots a machine with the supplied BIOS and card image, then docks it. Returns nullptr and sets
  // error if the image is not a valid BIOS, or if the machine does not enable communication after
  // docking.
  //
  // flash holds the full 128 KB card image. The BIOS scans the flash directory during boot to
  // set the auto-start slot (RAM[0xCE]). The flash must contain the card data before boot so
  // that scan finds any app already on the card.
  //
  // slot is the memory card slot the device sits in. It picks the serial of the device, since two
  // devices do not share one.
  //
  // quicksave_path is optional. When non-empty, Create attempts to restore the machine state from
  // that file before docking. A successful restore replaces the BIOS boot sequence: the app picks
  // up exactly where it left off, including work RAM written by the previous session's 0x5C
  // dispatches. The restore is skipped when the file is absent or was written for a different card.
  static std::unique_ptr<PocketStation> Create(std::span<const u8> bios,
                                               const MemoryCardImage::DataArray& flash, u32 slot,
                                               const std::string& quicksave_path, Error* error);

  // Exchanges one byte with the console. Returns true when the device acknowledges.
  //
  // This runs the emulated CPU, since the reply comes from an interrupt handler on the device. A
  // false return is a missing acknowledge, which the console reads as the device being absent or
  // the command being complete.
  bool Transfer(u8 data_in, u8* data_out);

  // Releases the select line at the end of a command.
  //
  // was_accessed must be false when the device was not involved in the transaction (e.g. a
  // controller poll); in that case the select release is skipped since the device was never
  // selected.
  void ResetTransferState(bool was_accessed);

  // Copies the card image into the flash of the device, and back out again. The flash of a
  // PocketStation is the memory card storage, so these are the join between MemoryCard::m_data and
  // the emulated machine.
  void LoadFlash(const MemoryCardImage::DataArray& data);

  // Reads the flash back out, and returns true when it differs from what data already held. It
  // compares rather than tracking writes, because the device writes its own flash while an app
  // runs.
  bool SaveFlash(MemoryCardImage::DataArray* data);

  // Size of the machine state in bytes. Measuring it walks every field of the machine, and the core
  // keeps it constant for a given build, so it is measured once at construction instead.
  size_t GetStateSize() const { return m_state_size; }

  bool DoState(StateWrapper& sw);

  // Stops the ARM thread, triggers the hold-save, clears the docked flag, and clears the LCD
  // rotation bit. Call this before SaveFlash and ExportQuicksave so both see the final app state
  // and the exported state loads with standalone orientation in pokketstation.
  void PrepareForSave();

  // Copies the current 32x32 1bpp framebuffer into buf (128 bytes, 4 bytes per row, bit 0 is the
  // leftmost pixel, 0 = white, 1 = black). Returns true when the framebuffer changed since the
  // last call and buf was updated; returns false and leaves buf unchanged when there is no change.
  bool ReadFramebuffer(std::array<u8, PSEMU_LCD_WIDTH * PSEMU_LCD_HEIGHT / 8>& buf);

  // Writes a pokketstation-compatible quicksave (slot 0 format) to path. The file holds the full
  // machine state and can be loaded directly by the pokketstation desktop frontend when it opens
  // the same card image. Call after the hold-save sequence so flash holds the latest app data.
  void ExportQuicksave(const std::string& path) const;

private:
  void ARMThreadFunc();

  psemu* m_ps = nullptr;
  size_t m_state_size = 0;
  u32 m_slot = 0;
  u32 m_cmd_byte_pos = 0u;
  u8 m_cmd_in_progress = 0u;
  bool m_app_slot_set = false;

  // The ARM runs in a background thread. m_psemu_mutex protects all psemu state: the ARM thread
  // holds it during psemu_run, and the main thread holds it during every psemu call. m_wake_cv
  // lets ResetTransferState wake the ARM thread immediately after a SELECT release so the BIOS
  // end-of-command path runs without waiting for the next periodic tick.
  std::mutex m_psemu_mutex;
  std::mutex m_wake_mutex;
  std::condition_variable m_wake_cv;
  std::thread m_arm_thread;
  std::atomic<bool> m_arm_running{false};
};
