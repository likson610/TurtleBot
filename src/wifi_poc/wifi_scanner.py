import os
import re
import subprocess
import rclpy
from rclpy.node import Node
from wifi_msgs.msg import WiFiScan, WiFiNetwork

class WiFiScannerNode(Node):
    def __init__(self):
        super().__init__('wifi_scanner_node')
        
        self.interface = os.environ.get('WIFI_INTERFACE', 'wlan1')
        self.freqs = os.environ.get('SCAN_FREQUENCIES', '').strip()
        
        # Publikacja dedykowanego typu wiadomości
        self.publisher_ = self.create_publisher(WiFiScan, '/wifi/scan', 10)
        
        timer_period = 1.0 if self.freqs else 3.0
        self.timer = self.create_timer(timer_period, self.scan_and_publish)
        
        print(f"[START] Węzeł wifi_scanner uruchomiony na interfejsie {self.interface}", flush=True)

    def scan_and_publish(self):
        cmd = ["iw", "dev", self.interface, "scan"]
        if self.freqs:
            cmd.extend(["freq"] + self.freqs.split())

        try:
            result = subprocess.run(cmd, capture_output=True, text=True, check=True)
            networks = self._parse_iw_scan(result.stdout)
            
            scan_msg = WiFiScan()
            scan_msg.header.stamp = self.get_clock().now().to_msg()
            scan_msg.header.frame_id = "base_link"
            scan_msg.networks = networks
            
            self.publisher_.publish(scan_msg)
            print(f"[SCAN] Opublikowano {len(networks)} stacji na /wifi/scan", flush=True)
            
        except subprocess.CalledProcessError as e:
            if "Device or resource busy" in e.stderr:
                print("[WARN] Interfejs zajęty innym skanem, ponawianie...", flush=True)
            else:
                print(f"[ERROR iw]: {e.stderr.strip()}", flush=True)
        except Exception as ex:
            print(f"[ERROR]: {ex}", flush=True)

    def _parse_iw_scan(self, raw_output):
        networks = []
        current_net = None

        for line in raw_output.splitlines():
            line = line.strip()
            
            bss_match = re.match(r'^BSS\s+([0-9a-fA-F:]{17})', line)
            if bss_match:
                if current_net:
                    networks.append(current_net)
                current_net = WiFiNetwork()
                current_net.bssid = bss_match.group(1).lower()
                current_net.rssi_dbm = 0.0
                current_net.frequency_mhz = 0.0
                current_net.channel = 0
                continue

            if not current_net:
                continue

            freq_match = re.match(r'^freq:\s+(\d+)', line)
            if freq_match:
                freq = int(freq_match.group(1))
                current_net.frequency_mhz = float(freq)
                # Proste wyliczenie kanału w paśmie 2.4 GHz
                if 2412 <= freq <= 2472:
                    current_net.channel = int((freq - 2407) / 5)
                elif freq == 2484:
                    current_net.channel = 14

            signal_match = re.match(r'^signal:\s+([-\d.]+)\s*dBm', line)
            if signal_match:
                current_net.rssi_dbm = float(signal_match.group(1))

            ssid_match = re.match(r'^SSID:\s*(.*)', line)
            if ssid_match:
                current_net.ssid = ssid_match.group(1)

        if current_net:
            networks.append(current_net)

        return networks

def main(args=None):
    rclpy.init(args=args)
    node = WiFiScannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()