#include "tetra_dsv_s_bringup/drive_board.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace tetra_dsv_s_bringup
{

namespace
{
constexpr uint8_t STX = 0x02;
constexpr uint8_t ETX = 0x03;
constexpr size_t MAX_FRAME = 64;
// Gap between bytes that means "the board's burst is over", once at least
// one byte has arrived. Real inter-byte gaps inside a burst are ~87us at
// 115200 baud, so this has ample margin without adding real latency to the
// 30Hz control loop.
constexpr int IDLE_GAP_MS = 5;
}  // namespace

DriveBoard::~DriveBoard()
{
  close();
}

bool DriveBoard::open(const std::string & device, int read_timeout_ms)
{
  close();
  read_timeout_ms_ = read_timeout_ms;

  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (fd_ < 0) {
    last_error_ = "open(" + device + "): " + std::strerror(errno);
    return false;
  }

  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    last_error_ = std::string("tcgetattr: ") + std::strerror(errno);
    close();
    return false;
  }

  cfsetospeed(&tty, B115200);
  cfsetispeed(&tty, B115200);

  // 8N1, no flow control, fully raw. cfmakeraw is not used so the intent of
  // each flag stays visible - a stray ICANON here would make reads block until
  // a newline that this binary protocol never sends.
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL |
    IXON | IXOFF | IXANY);
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG);
  tty.c_oflag &= ~(OPOST | ONLCR);

  // VMIN 0 + VTIME in decisecconds: a read returns as soon as any byte is
  // available, or after the timeout with nothing. Never blocks forever.
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = static_cast<cc_t>((read_timeout_ms_ + 99) / 100);

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    last_error_ = std::string("tcsetattr: ") + std::strerror(errno);
    close();
    return false;
  }

  tcflush(fd_, TCIOFLUSH);
  last_error_.clear();
  return true;
}

void DriveBoard::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

uint8_t DriveBoard::lrc(const uint8_t * data, size_t len)
{
  uint8_t acc = 0;
  for (size_t i = 0; i < len; ++i) {
    acc ^= data[i];
  }
  return acc;
}

void DriveBoard::encode_speed(int mm_s, uint8_t & hi, uint8_t & lo)
{
  if (mm_s < 0) {
    const unsigned mag = static_cast<unsigned>(-mm_s);
    hi = static_cast<uint8_t>((mag >> 8) | 0x80);
    lo = static_cast<uint8_t>(mag & 0xff);
  } else {
    const unsigned mag = static_cast<unsigned>(mm_s);
    hi = static_cast<uint8_t>(mag >> 8);
    lo = static_cast<uint8_t>(mag & 0xff);
  }
}

bool DriveBoard::write_frame(const uint8_t * buf, size_t len)
{
  if (fd_ < 0) {
    last_error_ = "port not open";
    return false;
  }
  size_t written = 0;
  while (written < len) {
    const ssize_t n = ::write(fd_, buf + written, len - written);
    if (n <= 0) {
      if (errno == EINTR) {continue;}
      last_error_ = std::string("write: ") + std::strerror(errno);
      return false;
    }
    written += static_cast<size_t>(n);
  }
  return true;
}

int DriveBoard::read_frame(uint8_t * buf, size_t max_len)
{
  if (fd_ < 0) {
    last_error_ = "port not open";
    return -1;
  }
  size_t got = 0;
  while (got < max_len) {
    // Frame end is judged by a quiet gap on the wire, not by matching ETX
    // (0x03) as a byte value: that value recurs constantly inside the
    // payload (e.g. theta_raw's high byte), which used to truncate BV
    // replies before they could be decoded. The board sends each reply as
    // one burst, so any gap this size past the first byte means it's done.
    struct pollfd pfd{fd_, POLLIN, 0};
    const int wait_ms = (got == 0) ? read_timeout_ms_ : IDLE_GAP_MS;
    const int pr = ::poll(&pfd, 1, wait_ms);
    if (pr < 0) {
      if (errno == EINTR) {continue;}
      last_error_ = std::string("poll: ") + std::strerror(errno);
      return -1;
    }
    if (pr == 0) {
      if (got == 0) {
        last_error_ = "read timeout";
        return -1;
      }
      return static_cast<int>(got);
    }

    const ssize_t n = ::read(fd_, buf + got, 1);
    if (n < 0) {
      if (errno == EINTR) {continue;}
      last_error_ = std::string("read: ") + std::strerror(errno);
      return -1;
    }
    if (n == 0) {continue;}  // spurious wakeup, retry
    // Resynchronise: ignore anything before STX so a half-consumed reply from
    // a previous cycle cannot shift every field by a byte.
    if (got == 0 && buf[0] != STX) {continue;}
    ++got;
  }
  last_error_ = "frame overrun";
  return -1;
}

bool DriveBoard::reset_error()
{
  uint8_t p[5] = {STX, 'C', 'G', ETX, 0};
  p[4] = lrc(&p[1], 3);
  if (!write_frame(p, 5)) {return false;}
  uint8_t rx[MAX_FRAME];
  return read_frame(rx, MAX_FRAME) > 0;
}

bool DriveBoard::set_velocity_mode()
{
  uint8_t p[7] = {STX, 'C', 'Z', '1', '1', ETX, 0};
  p[6] = lrc(&p[1], 5);
  if (!write_frame(p, 7)) {return false;}
  uint8_t rx[MAX_FRAME];
  return read_frame(rx, MAX_FRAME) > 0;
}

bool DriveBoard::set_servo(bool on)
{
  const uint8_t c = on ? '1' : '0';
  uint8_t p[7] = {STX, 'D', 'B', c, c, ETX, 0};
  p[6] = lrc(&p[1], 5);
  if (!write_frame(p, 7)) {return false;}
  uint8_t rx[MAX_FRAME];
  return read_frame(rx, MAX_FRAME) > 0;
}

bool DriveBoard::set_velocity(int left_mm_s, int right_mm_s, DriveState & state)
{
  uint8_t p[9] = {STX, 'B', 'V', 0, 0, 0, 0, ETX, 0};
  encode_speed(left_mm_s, p[3], p[4]);
  encode_speed(right_mm_s, p[5], p[6]);
  p[8] = lrc(&p[1], 7);

  if (!write_frame(p, 9)) {return false;}

  uint8_t rx[MAX_FRAME] = {0};
  const int n = read_frame(rx, MAX_FRAME);
  if (n < 14) {
    // With E-stop engaged the board answers with a status-only frame
    // (02 32 03 LRC) and no odometry payload. That is a robot state, not a
    // protocol fault - say so, or the next person spends an afternoon
    // debugging the framing code like we did on 2026-08-13.
    if (n >= 4 && rx[1] == 0x32) {
      state.emergency = true;
      last_error_ = "emergency stop engaged at the robot";
    } else if (n >= 0) {
      last_error_ = "short BV reply";
    }
    return false;
  }

  // rx[1] doubles as the status byte: '0' normal, '2' emergency stop engaged.
  // Anything else means the frame is not the reply we expect.
  if (rx[1] != 0x30 && rx[1] != 0x32) {
    last_error_ = "bad BV status byte";
    return false;
  }

  // X and Y are 32-bit big-endian with bit 31 as a SIGN FLAG, not two's
  // complement - mask it off before negating or the magnitude is wrong.
  const uint32_t xb = (static_cast<uint32_t>(rx[2]) << 24 & 0x7f000000) |
    (static_cast<uint32_t>(rx[3]) << 16 & 0x00ff0000) |
    (static_cast<uint32_t>(rx[4]) << 8 & 0x0000ff00) |
    (static_cast<uint32_t>(rx[5]) & 0xff);
  state.x_mm = (rx[2] >> 7) ? -static_cast<int32_t>(xb) : static_cast<int32_t>(xb);

  const uint32_t yb = (static_cast<uint32_t>(rx[6]) << 24 & 0x7f000000) |
    (static_cast<uint32_t>(rx[7]) << 16 & 0x00ff0000) |
    (static_cast<uint32_t>(rx[8]) << 8 & 0x0000ff00) |
    (static_cast<uint32_t>(rx[9]) & 0xff);
  state.y_mm = (rx[6] >> 7) ? -static_cast<int32_t>(yb) : static_cast<int32_t>(yb);

  state.theta_raw = static_cast<int32_t>(
    (static_cast<uint32_t>(rx[10]) << 8 & 0xff00) | (static_cast<uint32_t>(rx[11]) & 0xff));
  state.bumper = rx[12];
  state.emergency = (rx[1] != 0x30);
  return true;
}

bool DriveBoard::reset_odometry()
{
  // Vendor sends set_odometry(0,0,0) as command "CO" with a zero payload.
  uint8_t p[16] = {STX, 'C', 'O', 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, ETX, 0};
  p[14] = lrc(&p[1], 13);
  if (!write_frame(p, 15)) {return false;}
  uint8_t rx[MAX_FRAME];
  return read_frame(rx, MAX_FRAME) > 0;
}

bool DriveBoard::read_parameter(int index, int & value)
{
  uint8_t p[7] = {STX, 'C', 'R', static_cast<uint8_t>(index), 0, ETX, 0};
  p[6] = lrc(&p[1], 5);
  if (!write_frame(p, 7)) {return false;}

  uint8_t rx[MAX_FRAME] = {0};
  const int n = read_frame(rx, MAX_FRAME);
  if (n < 6) {return false;}
  value = static_cast<int>((static_cast<uint32_t>(rx[2]) << 8 & 0xff00) |
    (static_cast<uint32_t>(rx[3]) & 0xff));
  return true;
}

}  // namespace tetra_dsv_s_bringup
