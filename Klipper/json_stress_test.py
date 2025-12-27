#!/usr/bin/env python3
"""
JSON Parser Stress Test for BMCU LiteJSON
Tests malformed inputs, edge cases, and verifies graceful failure.
"""

import serial
import json
import time
import random
import string

# Configuration
PORT = "COM3"  # Change as needed
BAUD = 115200
TIMEOUT = 1.0

class JSONStressTester:
    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=TIMEOUT)
        time.sleep(2)  # Wait for device reset
        self.passed = 0
        self.failed = 0
        
    def send_raw(self, data):
        """Send raw bytes and get response"""
        self.ser.write((data + "\n").encode())
        time.sleep(0.1)
        return self.ser.read_all().decode(errors='replace')
    
    def send_json(self, obj):
        """Send JSON object and get response"""
        return self.send_raw(json.dumps(obj))
    
    def test(self, name, data, expect_ok=None, expect_error=False):
        """Run a test case"""
        print(f"  [{name}]", end=" ")
        
        if isinstance(data, dict):
            resp = self.send_json(data)
        else:
            resp = self.send_raw(data)
        
        # Check for crash (no response)
        if not resp.strip():
            print("❌ NO RESPONSE (FREEZE?)")
            self.failed += 1
            return False
        
        # Parse response
        try:
            r = json.loads(resp.split('\n')[0])
            got_ok = r.get("ok", False)
            got_error = "error" in r or r.get("ok") == False
            
            if expect_ok is not None:
                if got_ok == expect_ok:
                    print(f"✅ ok={got_ok}")
                    self.passed += 1
                    return True
                else:
                    print(f"❌ Expected ok={expect_ok}, got {got_ok}")
                    self.failed += 1
                    return False
            elif expect_error:
                if got_error:
                    print(f"✅ Graceful error: {r.get('msg', r.get('code', 'error'))}")
                    self.passed += 1
                    return True
                else:
                    print(f"❌ Expected error, got: {resp[:50]}")
                    self.failed += 1
                    return False
            else:
                print(f"✅ Response: {resp[:60]}...")
                self.passed += 1
                return True
                
        except json.JSONDecodeError:
            if expect_error:
                print(f"✅ Malformed response (expected): {resp[:40]}...")
                self.passed += 1
                return True
            print(f"❌ Invalid JSON response: {resp[:60]}")
            self.failed += 1
            return False
    
    def run_all(self):
        """Run all test suites"""
        print("\n" + "="*60)
        print("BMCU LiteJSON Stress Test")
        print("="*60)
        
        # === Valid Commands ===
        print("\n[1] Valid Commands")
        self.test("PING", {"id": 1, "cmd": "PING", "args": {}}, expect_ok=True)
        self.test("STATUS", {"id": 2, "cmd": "STATUS", "args": {}}, expect_ok=True)
        self.test("GET_FILAMENT_INFO", {"id": 3, "cmd": "GET_FILAMENT_INFO", "args": {"lane": 0}}, expect_ok=True)
        
        # === Malformed JSON ===
        print("\n[2] Malformed JSON (should gracefully error)")
        self.test("Missing brace", '{"id": 1, "cmd": "PING"', expect_error=True)
        self.test("Extra comma", '{"id": 1, "cmd": "PING",}', expect_error=True)
        self.test("Single quotes", "{'id': 1, 'cmd': 'PING'}", expect_error=True)
        self.test("No quotes on key", '{id: 1, cmd: "PING"}', expect_error=True)
        self.test("Empty string", '', expect_error=True)
        self.test("Just whitespace", '   \t\n  ', expect_error=True)
        self.test("Random garbage", 'asdfgh12345!@#$%', expect_error=True)
        self.test("Binary garbage", '\x00\x01\x02\x03\x04', expect_error=True)
        
        # === Edge Cases ===
        print("\n[3] Edge Cases")
        self.test("Empty object", '{}', expect_error=True)
        self.test("No cmd", '{"id": 1}', expect_error=True)
        self.test("Unknown cmd", {"id": 1, "cmd": "INVALID_COMMAND", "args": {}}, expect_error=True)
        self.test("Negative ID", {"id": -999, "cmd": "PING", "args": {}}, expect_ok=True)
        self.test("String ID", {"id": "not_a_number", "cmd": "PING", "args": {}}, expect_ok=True)
        
        # === String Length Tests ===
        print("\n[4] String Length Limits")
        self.test("Long cmd (50 chars)", {"id": 1, "cmd": "A"*50, "args": {}}, expect_error=True)
        self.test("Long string value", {"id": 1, "cmd": "SET_FILAMENT_INFO", "args": {"lane": 0, "name": "X"*100}}, expect_error=True)
        
        # === Nesting Depth ===
        print("\n[5] Nesting Depth")
        self.test("Normal nesting", {"id": 1, "cmd": "STATUS", "args": {"nested": {"deep": 1}}}, expect_error=True)
        
        # === Array Tests ===
        print("\n[6] Array Tests")
        self.test("Valid color array", {"id": 1, "cmd": "SET_FILAMENT_INFO", "args": {"lane": 0, "color": [255, 128, 0]}})
        self.test("Large array (10 elements)", {"id": 1, "cmd": "SET_FILAMENT_INFO", "args": {"lane": 0, "color": [1,2,3,4,5,6,7,8,9,10]}})
        self.test("Empty array", {"id": 1, "cmd": "SET_FILAMENT_INFO", "args": {"lane": 0, "color": []}})
        
        # === Numeric Tests ===
        print("\n[7] Numeric Edge Cases")
        self.test("Float meters", {"id": 1, "cmd": "SET_FILAMENT_INFO", "args": {"lane": 0, "meters": 99.99}})
        self.test("Zero meters", {"id": 1, "cmd": "SET_FILAMENT_INFO", "args": {"lane": 0, "meters": 0}})
        self.test("Negative meters", {"id": 1, "cmd": "SET_FILAMENT_INFO", "args": {"lane": 0, "meters": -10.5}})
        self.test("Large int", {"id": 1, "cmd": "SET_FILAMENT_INFO", "args": {"lane": 0, "temp_min": 999999}})
        
        # === Rapid Fire ===
        print("\n[8] Rapid Fire (10 fast requests)")
        for i in range(10):
            self.test(f"Rapid #{i+1}", {"id": i, "cmd": "PING", "args": {}}, expect_ok=True)
        
        # === Summary ===
        print("\n" + "="*60)
        total = self.passed + self.failed
        print(f"Results: {self.passed}/{total} passed ({100*self.passed/total:.1f}%)")
        if self.failed == 0:
            print("✅ ALL TESTS PASSED!")
        else:
            print(f"❌ {self.failed} tests failed")
        print("="*60)
        
        self.ser.close()

if __name__ == "__main__":
    import sys
    port = sys.argv[1] if len(sys.argv) > 1 else PORT
    print(f"Using port: {port}")
    
    tester = JSONStressTester(port, BAUD)
    tester.run_all()
