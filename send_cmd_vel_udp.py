#!/usr/bin/env python3
import argparse
import json
import socket
import time


def main():
    parser = argparse.ArgumentParser(description="Send /cmd_vel UDP commands to Unity ExternalControlManager")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9001)
    parser.add_argument("--linear", type=float, default=0.0, help="Forward command [-1..1]")
    parser.add_argument("--angular", type=float, default=0.0, help="Turn command [-1..1]")
    parser.add_argument("--topic", default="/cmd_vel")
    parser.add_argument("--rate", type=float, default=15.0, help="Send rate Hz")
    parser.add_argument("--duration", type=float, default=3.0, help="Duration seconds")
    args = parser.parse_args()

    pkt = {"topic": args.topic, "linear": args.linear, "angular": args.angular}
    data = json.dumps(pkt).encode("utf-8")
    period = 1.0 / max(args.rate, 1.0)
    deadline = time.time() + max(args.duration, 0.0)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sent = 0
    while time.time() < deadline:
        sock.sendto(data, (args.host, args.port))
        sent += 1
        time.sleep(period)
    sock.close()
    print(f"Sent {sent} packets to {args.host}:{args.port} -> {pkt}")


if __name__ == "__main__":
    main()

