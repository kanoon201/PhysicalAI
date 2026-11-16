"""
ESC / servo bench test -- no camera, no vision, just the control link.

    python esc_test.py --esp-ip 192.168.x.x

Sends neutral for 3 s (so the ESC can arm), then steers left/right so
you can see the servo, then ramps throttle 0 -> 0.6 slowly and back.
LIFT THE WHEELS OFF THE GROUND FIRST. Ctrl+C stops it at any point.
"""
import argparse
import time

from robot_link import RobotLink


def hold(link, steer, throttle, secs, label):
    print(f"{label:<28} steer {steer:+.2f}  throttle {throttle:+.2f}  ({secs:.1f}s)")
    end = time.time() + secs
    while time.time() < end:
        link.drive(steer, throttle)
        time.sleep(0.02)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--esp-ip", required=True)
    ap.add_argument("--max", type=float, default=0.6, help="peak throttle (default 0.6)")
    ap.add_argument("--reverse", action="store_true", help="also try negative throttle")
    args = ap.parse_args()

    link = RobotLink(args.esp_ip)
    link.start()
    try:
        hold(link, 0.0, 0.0, 3.0, "neutral (ESC arming)")
        hold(link, -0.5, 0.0, 1.0, "steer LEFT")
        hold(link, +0.5, 0.0, 1.0, "steer RIGHT")
        hold(link, 0.0, 0.0, 1.0, "center")
        for t in [0.2, 0.3, 0.4, 0.5, args.max]:
            hold(link, 0.0, t, 1.5, f"throttle {t:.1f}")
        hold(link, 0.0, 0.0, 1.0, "neutral")
        if args.reverse:
            for t in [-0.2, -0.3, -0.4]:
                hold(link, 0.0, t, 1.5, f"throttle {t:.1f}")
        hold(link, 0.0, 0.0, 1.0, "neutral")
        print(f"done, {link.packets_sent} packets sent")
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        link.stop()
        link.close()


if __name__ == "__main__":
    main()
