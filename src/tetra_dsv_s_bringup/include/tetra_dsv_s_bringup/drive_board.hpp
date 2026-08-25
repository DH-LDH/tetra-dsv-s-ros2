// RS-232 link to the TETRA-DSV-S drive board (CN20, 115200 8N1).
//
// Ported from the vendor's ROS 1 sources, which are the only documentation of
// the wire format that actually matches the shipped firmware:
//     TETRA-DSV-S/tetraDS/src/drive_module.c
//     TETRA-DSV-S/tetraDS/include/rs232_common.c
//
// Frame layout - every command is the same shape:
//
//     STX  cmd0 cmd1  [payload...]  ETX  LRC
//     0x02                          0x03
//
// LRC is a plain XOR over everything between STX and ETX inclusive of the
// command bytes, i.e. bytes [1 .. n-1] of the buffer. Not a checksum, not a
// CRC - XOR. Getting this wrong is the single easiest way to have the board
// ignore you in silence.
//
// The one command that matters for Nav2 is BV (setVelocity2): it carries the
// wheel speeds out and brings odometry, bumper and e-stop back in the SAME
// round trip. At 115200 the number of round trips per cycle is what caps the
// control rate, so never split this into separate command + query calls.

#ifndef TETRA_DSV_S_BRINGUP__DRIVE_BOARD_HPP_
#define TETRA_DSV_S_BRINGUP__DRIVE_BOARD_HPP_

#include <cstdint>
#include <string>

namespace tetra_dsv_s_bringup
{

/// One BV response: firmware-integrated pose plus the safety inputs.
struct DriveState
{
  int32_t x_mm = 0;        ///< firmware odometry, millimetres
  int32_t y_mm = 0;
  int32_t theta_raw = 0;   ///< firmware heading, raw 16-bit
  uint8_t bumper = 0;      ///< bumper bitfield, 0 = clear
  bool emergency = false;  ///< true when the board reports E-stop
};

class DriveBoard
{
public:
  DriveBoard() = default;
  ~DriveBoard();

  DriveBoard(const DriveBoard &) = delete;
  DriveBoard & operator=(const DriveBoard &) = delete;

  /// Open the port and configure it 115200 8N1, raw, with a read timeout.
  /// Returns false and fills last_error() on failure.
  bool open(const std::string & device, int read_timeout_ms = 100);
  void close();
  bool is_open() const {return fd_ >= 0;}

  /// CG - clear latched EMG/bumper/motor error state on the board. Not just
  /// for a real E-stop press: a full power cycle of the robot chassis leaves
  /// the board in this same latched state (rx[1]==0x32, short status-only
  /// reply) with no button involved (2026-08-24, confirmed via probe_cg.py -
  /// CG alone flipped it back to 0x30 with the E-stop untouched). Call this
  /// before set_velocity_mode()/set_servo() on every startup, not just after
  /// a real E-stop press.
  bool reset_error();

  /// CZ11 - required before BV will move anything. The board boots in position
  /// mode; sending velocities without this is accepted and silently ignored.
  bool set_velocity_mode();

  /// DB - servo on/off. Off leaves the wheels free.
  bool set_servo(bool on);

  /// BV - command wheel speeds in mm/s and read back state in one round trip.
  bool set_velocity(int left_mm_s, int right_mm_s, DriveState & state);

  /// Zero the firmware's integrated pose.
  bool reset_odometry();

  /// Read one drive-board parameter (see manual table 4-3). Para4 is the wheel
  /// diameter in mm, Para5 the wheel separation in mm - worth reading on a real
  /// robot, because the manual and the vendor driver disagree about Para5.
  bool read_parameter(int index, int & value);

  const std::string & last_error() const {return last_error_;}

private:
  bool write_frame(const uint8_t * buf, size_t len);
  /// Read until ETX or timeout. Returns bytes stored, or -1.
  int read_frame(uint8_t * buf, size_t max_len);

  static uint8_t lrc(const uint8_t * data, size_t len);
  /// Wheel speeds are magnitude + sign bit in the high byte's bit 7, NOT two's
  /// complement. -500 is 0x81F4, not 0xFE0C.
  static void encode_speed(int mm_s, uint8_t & hi, uint8_t & lo);

  int fd_ = -1;
  int read_timeout_ms_ = 100;
  std::string last_error_;
};

}  // namespace tetra_dsv_s_bringup

#endif  // TETRA_DSV_S_BRINGUP__DRIVE_BOARD_HPP_
