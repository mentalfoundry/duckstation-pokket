// SPDX-FileCopyrightText: 2019-2026 Connor McLaughlin <stenzek@gmail.com>
// SPDX-License-Identifier: CC-BY-NC-ND-4.0

#pragma once

#include "memory_card_image.h"

#include "common/types.h"

#include <memory>
#include <span>

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
  static std::unique_ptr<PocketStation> Create(std::span<const u8> bios,
                                               const MemoryCardImage::DataArray& flash, u32 slot,
                                               Error* error);

  // Resets the machine and re-runs the boot and dock sequence with the supplied card image. Used
  // when an app is downloaded to the card while the device is running: the BIOS scans the
  // directory at boot to set the auto-start slot (RAM[0xCE]), and that scan needs to see the
  // new app. This is the emulation of a physical disconnect and reconnect.
  //
  // Returns true when communication is re-enabled after docking. A false return leaves the device
  // in a non-communicating state; the caller should log and continue.
  bool Reboot(const MemoryCardImage::DataArray& flash);

  // Exchanges one byte with the console. Returns true when the device acknowledges.
  //
  // This runs the emulated CPU, since the reply comes from an interrupt handler on the device. A
  // false return is a missing acknowledge, which the console reads as the device being absent or
  // the command being complete.
  bool Transfer(u8 data_in, u8* data_out);

  // Releases the select line at the end of a command. When was_accessed is true, also runs the
  // settle frames so the device can act on the release: its BIOS waits for this release after the
  // last byte, and without it the device answers one command and then answers nothing.
  //
  // was_accessed must be false when the device was not involved in the transaction (e.g. a
  // controller poll); in that case the settle frames are skipped to avoid burning ARM interpreter
  // time on every SELECT deassert regardless of what was addressed.
  void ResetTransferState(bool was_accessed);

  // Copies the card image into the flash of the device, and back out again. The flash of a
  // PocketStation is the memory card storage, so these are the join between MemoryCard::m_data and
  // the emulated machine.
  void LoadFlash(const MemoryCardImage::DataArray& data);

  // Reads the flash back out, and returns true when it differs from what data already held. It
  // compares rather than tracking writes, because the device writes its own flash while an app
  // runs.
  //
  // When needs_reboot is non-null and the flash change shows a newly-complete PocketStation app
  // chain in the directory (MCX0 type, chain intact) that was absent or incomplete before, it is
  // set to true. The caller should then call Reboot() so the BIOS picks up the new app.
  bool SaveFlash(MemoryCardImage::DataArray* data, bool* needs_reboot = nullptr) const;

  // Size of the machine state in bytes. Measuring it walks every field of the machine, and the core
  // keeps it constant for a given build, so it is measured once at construction instead.
  size_t GetStateSize() const { return m_state_size; }

  bool DoState(StateWrapper& sw);

private:
  psemu* m_ps = nullptr;
  size_t m_state_size = 0;
  u32 m_slot = 0;
  u32 m_cmd_byte_pos = 0u;
  u8 m_cmd_in_progress = 0u;
};
