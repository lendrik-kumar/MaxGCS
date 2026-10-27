"""Share one flight-controller port between MaxGCS and the tracker (a tiny MAVLink port splitter).

A serial port can only be opened by one program. This opens it once and relays it over UDP, so MaxGCS
and visual_track.py / fc_check.py can both talk to the same FC at the same time:

    FC (COM11) <--serial--> fc_bridge.py <--UDP 14570--> MaxGCS      (UDP, listen on port 14550)
                                         \--UDP 14570--> tracker     (--connect udpin:0.0.0.0:14551)

Everything the FC sends is copied to every client; everything any client sends goes to the FC. MAVLink
is packet-based and every client has its own system/component id, so no parsing is needed.

    python fc_bridge.py --fc COM11,115200
    python fc_bridge.py --fc COM11,115200 --out 127.0.0.1:14550 127.0.0.1:14551
    python fc_bridge.py --fc udp:127.0.0.1:14560          # upstream over UDP (the simulated FC, for testing)

Only one program may own the FC port: close Mission Planner / QGC first.
"""
import argparse
import select
import socket
import sys
import time

try:
    import serial  # pyserial
except ImportError:
    serial = None


class Upstream:
    """The FC side: a serial port ('COM11,115200') or a UDP peer ('udp:host:port')."""

    def __init__(self, spec):
        self.udp = spec.lower().startswith("udp:")
        if self.udp:
            host, port = spec[4:].rsplit(":", 1)
            self.peer = (host, int(port))
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            if hasattr(socket, "SIO_UDP_CONNRESET"):  # Windows: ignore ICMP "port unreachable" resets
                self.sock.ioctl(socket.SIO_UDP_CONNRESET, False)
            self.sock.bind(("127.0.0.1", 0))
            self.sock.setblocking(False)
        else:
            if serial is None:
                sys.exit("pyserial is not installed: pip install pyserial")
            port, _, baud = spec.partition(",")
            try:
                self.ser = serial.Serial(port, int(baud or 115200), timeout=0, write_timeout=1)
            except (serial.SerialException, OSError) as e:
                sys.exit(f"cannot open {port}: {e}\n(is another program - Mission Planner, QGC, MaxGCS - using it?)")

    def read(self):
        if self.udp:
            try:
                return self.sock.recvfrom(4096)[0]
            except (BlockingIOError, ConnectionResetError, OSError):
                return b""
        try:
            n = self.ser.in_waiting
            return self.ser.read(n) if n else b""
        except (serial.SerialException, OSError) as e:
            sys.exit(f"FC port lost: {e}")

    def write(self, data):
        try:
            if self.udp:
                self.sock.sendto(data, self.peer)
            else:
                self.ser.write(data)
        except (serial.SerialException, OSError) as e:
            if not self.udp:
                sys.exit(f"FC port lost: {e}")

    def close(self):
        (self.sock if self.udp else self.ser).close()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--fc", required=True, help="'COM11,115200' (serial) or 'udp:127.0.0.1:14560'")
    ap.add_argument("--out", nargs="+", default=["127.0.0.1:14550", "127.0.0.1:14551"], metavar="HOST:PORT",
                    help="where clients listen (default: MaxGCS UDP 14550 and the tracker 14551)")
    ap.add_argument("--listen", type=int, default=14570, help="UDP port clients send to / hear from (default 14570)")
    ap.add_argument("--quiet", action="store_true", help="no periodic byte counts")
    a = ap.parse_args(argv)

    outs = [(h, int(p)) for h, p in (o.rsplit(":", 1) for o in a.out)]
    up = Upstream(a.fc)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    if hasattr(socket, "SIO_UDP_CONNRESET"):
        sock.ioctl(socket.SIO_UDP_CONNRESET, False)
    sock.bind(("0.0.0.0", a.listen))
    sock.setblocking(False)
    print(f"[bridge] FC {a.fc}  <->  UDP :{a.listen}  ->  clients {', '.join(a.out)}", flush=True)

    n_up = n_down = 0
    t_rep = time.monotonic()
    try:
        while True:
            busy = False
            data = up.read()
            if data:
                busy = True
                n_up += len(data)
                for o in outs:
                    try:
                        sock.sendto(data, o)
                    except OSError:
                        pass
            r, _, _ = select.select([sock], [], [], 0)
            while r:
                try:
                    pkt = sock.recvfrom(4096)[0]
                except (BlockingIOError, ConnectionResetError, OSError):
                    break
                busy = True
                n_down += len(pkt)
                up.write(pkt)
                r, _, _ = select.select([sock], [], [], 0)
            if not busy:
                time.sleep(0.002)
            if not a.quiet and time.monotonic() - t_rep > 10:
                print(f"[bridge] from FC {n_up / 1000:.0f} kB   to FC {n_down / 1000:.1f} kB", flush=True)
                t_rep = time.monotonic()
    except KeyboardInterrupt:
        print("\n[bridge] stopped", flush=True)
    finally:
        up.close()
        sock.close()


if __name__ == "__main__":
    main()
