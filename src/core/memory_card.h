// SPDX-FileCopyrightText: 2019-2024 Connor McLaughlin <stenzek@gmail.com>
// SPDX-License-Identifier: CC-BY-NC-ND-4.0

#pragma once

#include "controller.h"
#include "memory_card_image.h"
#include "timing_event.h"

#include "common/bitfield.h"

#include <array>
#include <memory>
#include <span>
#include <string>
#include <string_view>

class Error;
class PocketStation;

class MemoryCard final
{
public:
  MemoryCard(u32 index);
  ~MemoryCard();

  // Fixed part of DoState(): the transfer state, plus the length field that precedes the payload.
  static constexpr u32 STATE_HEADER_SIZE = 1 + 1 + 2 + 1 + 1 + 1 + 4 + 1;

  // Serialized size of this card in bytes, which is not the same for every card: a slot holding a
  // PocketStation writes the machine state of the device in place of the raw image, since the flash
  // of that device is the image. Pad sizes its backup buffer from the sum of these.
  u32 GetStateSize() const;

  static std::unique_ptr<MemoryCard> Create(u32 index);
  static std::unique_ptr<MemoryCard> Open(u32 index, std::string path);

  const MemoryCardImage::DataArray& GetData() const { return m_data; }
  MemoryCardImage::DataArray& GetData() { return m_data; }
  const std::string& GetPath() const { return m_path; }

  // Turns this slot into a PocketStation, using the supplied BIOS image. The card image this slot
  // already holds becomes the flash of the device, since that flash is the card storage.
  //
  // Returns false and sets error if the image is not a valid BIOS, leaving the slot an ordinary
  // card.
  bool AttachPocketStation(std::span<const u8> bios, Error* error);

  bool HasPocketStation() const { return static_cast<bool>(m_pocketstation); }

  void Reset();
  bool DoState(StateWrapper& sw);
  void CopyState(const MemoryCard* src);

  void ResetTransferState();
  bool Transfer(const u8 data_in, u8* data_out);

  bool IsOrWasRecentlyWriting() const;

  void Format();

private:
  static constexpr u16 ADDRESS_MASK = 0x3FF;
  static constexpr u8 OFFSET_MASK = 0x7F;

  // save in three seconds, that should be long enough for everything to finish writing
  static constexpr u32 SAVE_DELAY_IN_SECONDS = 5;

  union FLAG
  {
    u8 bits;

    BitField<u8, bool, 3, 1> no_write_yet;
    BitField<u8, bool, 2, 1> write_error;
  };

  enum class State : u8
  {
    Idle,
    Command,

    ReadCardID1,
    ReadCardID2,
    ReadAddressMSB,
    ReadAddressLSB,
    ReadACK1,
    ReadACK2,
    ReadConfirmAddressMSB,
    ReadConfirmAddressLSB,
    ReadData,
    ReadChecksum,
    ReadEnd,

    WriteCardID1,
    WriteCardID2,
    WriteAddressMSB,
    WriteAddressLSB,
    WriteData,
    WriteChecksum,
    WriteACK1,
    WriteACK2,
    WriteEnd,

    GetIDCardID1,
    GetIDCardID2,
    GetIDACK1,
    GetIDACK2,
    GetID1,
    GetID2,
    GetID3,
    GetID4,
  };

  static TickCount GetSaveDelayInTicks();

  static std::string GetOSDMessageKey(u32 index);

  bool SaveIfChanged(bool display_osd_message);
  void QueueFileSave();

  State m_state = State::Idle;
  FLAG m_FLAG = {};
  u16 m_address = 0;
  u8 m_sector_offset = 0;
  u8 m_checksum = 0;
  u8 m_last_byte = 0;
  bool m_changed = false;

  TimingEvent m_save_event;
  std::string m_path;
  u32 m_index;

  MemoryCardImage::DataArray m_data{};

  // Set when this slot holds a PocketStation instead of an ordinary card.
  //
  // When it is set, the state machine above is not used at all. The device answers every command
  // itself, including the three standard ones, because its BIOS owns the protocol. m_data is then a
  // copy of the device's flash, refreshed at each command boundary: the device writes its own flash
  // while an app runs, so the contents can change with no console write to observe.
  std::unique_ptr<PocketStation> m_pocketstation;
};
