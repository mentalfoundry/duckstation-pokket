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

// A PocketStation in a memory card slot.
//
// The PocketStation is a memory card with an ARM7 machine inside it. It answers the three standard
// memory card commands (0x52 Read Sector, 0x53 Get ID, 0x57 Write Sector) and the PocketStation
// commands (0x50, and 0x58 to 0x5F).
//
// THE PROTOCOL IS NOT HERE, AND IT IS NOT IN THE CORE EITHER. The BIOS of the device holds it. A
// byte from the console raises an interrupt on the emulated ARM, and the FIQ handler of that BIOS
// selects the reply. Commands 0x5B and 0x5C go further: they execute a function number, and the
// numbers 0x80 to 0xFF resolve through a function table in the header of the app file. Thus no
// protocol code outside the emulated machine can answer them, and this class implements no command.
// It moves bytes, and it runs the machine.
//
// Requires a PocketStation BIOS image. Without one, nothing answers a transfer.
class PocketStation
{
public:
  PocketStation();
  ~PocketStation();

  // Boots a machine with the supplied BIOS, and docks it. Returns nullptr and sets error if the
  // image is not a valid BIOS, or if the machine does not enable communication after docking.
  static std::unique_ptr<PocketStation> Create(std::span<const u8> bios, Error* error);

  // Exchanges one byte with the console. Returns true when the device acknowledges.
  //
  // This RUNS THE EMULATED CPU. An interrupt handler on the device produces the reply, so the
  // machine has to execute before a reply exists. A false return means the device did not answer
  // inside its cycle budget, which the console reads the same way it reads a missing acknowledge
  // from an ordinary card: the device is absent, or the command is complete.
  bool Transfer(u8 data_in, u8* data_out);

  // Releases the select line at the end of a command, and runs the machine so the device can act on
  // it.
  //
  // THE DEVICE NEEDS THIS. Its BIOS learns that a command ended from the release of this line, and
  // it waits for that release after the last byte. Without the release it answers one command and
  // then answers nothing.
  void ResetTransferState();

  // Copies the card image into the flash of the device, and back out again.
  //
  // The flash of a PocketStation IS the memory card storage. The same 128KB holds the directory and
  // the saves that the console reads, and the app that the device executes. Thus these two calls are
  // the join between MemoryCard::m_data and the emulated machine.
  void LoadFlash(const MemoryCardImage::DataArray& data);

  // Reads the flash back out, and returns true when it differs from what data already held.
  //
  // The comparison is deliberate. The device writes its own flash while an app runs, so a change can
  // happen with no console write at all, and there is no write path here to hang a dirty flag on.
  bool SaveFlash(MemoryCardImage::DataArray* data) const;

  // Size of the machine state in bytes. The core keeps this constant for a given build, so it can
  // be measured before the machine is serialized.
  size_t GetStateSize() const;

  bool DoState(StateWrapper& sw);

private:
  psemu* m_ps = nullptr;
};
