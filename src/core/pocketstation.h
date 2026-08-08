// SPDX-FileCopyrightText: 2019-2026 Connor McLaughlin <stenzek@gmail.com>
// SPDX-License-Identifier: CC-BY-NC-ND-4.0

#pragma once

#include "memory_card_image.h"

#include "common/types.h"

#include <memory>
#include <span>
#include <string>
#include <string_view>

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
  // Size a BIOS image has to be. Anything else is not one.
  static constexpr u32 BIOS_SIZE = 16 * 1024;

  PocketStation();
  ~PocketStation();

  // Boots a machine with the supplied BIOS, and docks it. Returns nullptr and sets error if the
  // image is not a valid BIOS, or if the machine does not enable communication after docking.
  //
  // hardware_id is the serial of the device, as 8 hex digits. An empty or unparseable value falls
  // back to a default.
  static std::unique_ptr<PocketStation> Create(std::span<const u8> bios, std::string_view hardware_id, Error* error);

  // Canonical 8-hex-digit form of a hardware ID.
  static std::string FormatHardwareId(u32 id);

  // Default serial for a slot. Each slot gets a different one, since two devices do not share a
  // serial.
  static std::string GetDefaultHardwareId(u32 slot);

  // Exchanges one byte with the console. Returns true when the device acknowledges.
  //
  // This runs the emulated CPU, since the reply comes from an interrupt handler on the device. A
  // false return is a missing acknowledge, which the console reads as the device being absent or
  // the command being complete.
  bool Transfer(u8 data_in, u8* data_out);

  // Releases the select line at the end of a command, and runs the machine so the device can act on
  // it. Its BIOS learns that a command ended from this release and waits for it after the last
  // byte, so without it the device answers one command and then answers nothing.
  void ResetTransferState();

  // Copies the card image into the flash of the device, and back out again. The flash of a
  // PocketStation is the memory card storage, so these are the join between MemoryCard::m_data and
  // the emulated machine.
  void LoadFlash(const MemoryCardImage::DataArray& data);

  // Reads the flash back out, and returns true when it differs from what data already held. It
  // compares rather than tracking writes, because the device writes its own flash while an app
  // runs.
  bool SaveFlash(MemoryCardImage::DataArray* data) const;

  // Size of the machine state in bytes. Measuring it walks every field of the machine, and the core
  // keeps it constant for a given build, so it is measured once at construction instead.
  size_t GetStateSize() const { return m_state_size; }

  bool DoState(StateWrapper& sw);

private:
  psemu* m_ps = nullptr;
  size_t m_state_size = 0;
};
