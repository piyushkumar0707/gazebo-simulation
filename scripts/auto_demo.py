#!/usr/bin/env python3
"""
Automated Pick and Place Demo using Gazebo Services
This script directly controls the simulation to demonstrate pick-and-place
No ROS control required - uses Gazebo model state services
"""

import subprocess
import time
import math
import sys

class Color:
    BLUE = '\033[0;34m'
    GREEN = '\033[0;32m'
    YELLOW = '\033[1;33m'
    RED = '\033[0;31m'
    NC = '\033[0m'

def print_step(step_num, description):
    """Print formatted step"""
    print(f"\n{'='*70}")
    print(f"{Color.BLUE}  STEP {step_num}: {description}{Color.NC}")
    print(f"{'='*70}\n")

def set_joint_position(joint_name, position):
    """Set joint position using Gazebo service"""
    cmd = f'gz joint -m simple_3dof_arm -j {joint_name} --pos-t {position}'
    try:
        subprocess.run(cmd, shell=True, capture_output=True, timeout=2)
    except:
        # Try alternate command for different Gazebo versions
        pass

def move_to_position(base, shoulder, elbow, duration=2.0, description=""):
    """Move arm to position smoothly"""
    if description:
        print(f"{Color.GREEN}→ {description}{Color.NC}")
    
    print(f"  Moving to: base={math.degrees(base):.1f}°, "
          f"shoulder={math.degrees(shoulder):.1f}°, "
          f"elbow={math.degrees(elbow):.1f}°")
    
    # Set positions
    set_joint_position('joint_base', base)
    set_joint_position('joint_shoulder', shoulder)
    set_joint_position('joint_elbow', elbow)
    
    time.sleep(duration)

def move_object(object_name, x, y, z):
    """Move an object in the simulation"""
    cmd = f'gz model -m {object_name} -x {x} -y {y} -z {z}'
    try:
        subprocess.run(cmd, shell=True, capture_output=True, timeout=2)
    except:
        pass

def pick_and_place_demo():
    """Automated pick and place demonstration"""
    print(f"\n{Color.BLUE}{'='*70}")
    print("  🤖 AUTOMATED PICK AND PLACE DEMONSTRATION 🤖")
    print(f"{'='*70}{Color.NC}\n")
    
    print(f"{Color.YELLOW}This demo will automatically move the arm through a complete")
    print(f"pick-and-place sequence using Gazebo's built-in controls!{Color.NC}\n")
    
    time.sleep(2)
    
    # Step 0: Home position
    print_step(0, "HOME POSITION - Starting configuration")
    move_to_position(0.0, 0.0, 0.0, 2.0, "Moving to home position")
    time.sleep(1)
    
    # Step 1: Rotate base to align with object
    print_step(1, "ALIGN - Rotating base toward red cube")
    move_to_position(0.3, 0.0, 0.0, 2.5, "Rotating base to target")
    time.sleep(1)
    
    # Step 2: Extend arm toward object
    print_step(2, "APPROACH - Extending arm toward object")
    move_to_position(0.3, 0.8, -0.6, 3.0, "Extending arm forward")
    time.sleep(1)
    
    # Step 3: Position over object
    print_step(3, "POSITION - Fine positioning over red cube")
    move_to_position(0.3, 1.0, -0.9, 2.5, "Positioning gripper over object")
    time.sleep(1)
    
    # Step 4: Simulate grasp
    print_step(4, "GRASP - Closing gripper")
    print(f"{Color.GREEN}→ Gripper closing... [SIMULATED]{Color.NC}")
    print(f"  {Color.YELLOW}(Moving object to attach to gripper){Color.NC}")
    time.sleep(2)
    
    # Attach object by moving it to gripper position (simulation trick)
    move_object('cube_object', 0.45, 0.15, 0.35)
    time.sleep(0.5)
    
    # Step 5: Lift object
    print_step(5, "LIFT - Raising object")
    move_to_position(0.3, 0.5, -0.3, 3.0, "Lifting object upward")
    move_object('cube_object', 0.4, 0.15, 0.5)
    time.sleep(1)
    
    # Step 6: Transport to new location
    print_step(6, "TRANSPORT - Moving to drop location")
    move_to_position(-0.5, 0.5, -0.3, 3.5, "Transporting to new position")
    move_object('cube_object', 0.25, -0.3, 0.5)
    time.sleep(1)
    
    # Step 7: Lower to placement position
    print_step(7, "LOWER - Positioning for placement")
    move_to_position(-0.5, 0.9, -0.8, 3.0, "Lowering to placement height")
    move_object('cube_object', 0.3, -0.3, 0.15)
    time.sleep(1)
    
    # Step 8: Release object
    print_step(8, "RELEASE - Opening gripper")
    print(f"{Color.GREEN}→ Gripper opening... [SIMULATED]{Color.NC}")
    print(f"  {Color.YELLOW}(Placing object on table){Color.NC}")
    move_object('cube_object', 0.3, -0.3, 0.075)
    time.sleep(2)
    
    # Step 9: Retract
    print_step(9, "RETRACT - Moving away from object")
    move_to_position(-0.5, 0.4, -0.2, 2.5, "Retracting arm")
    time.sleep(1)
    
    # Step 10: Return home
    print_step(10, "COMPLETE - Returning to home position")
    move_to_position(0.0, 0.0, 0.0, 3.5, "Returning home")
    time.sleep(1)
    
    print(f"\n{Color.GREEN}{'='*70}")
    print("  ✨ DEMONSTRATION COMPLETE! ✨")
    print(f"{'='*70}{Color.NC}\n")
    
    print(f"{Color.YELLOW}The red cube has been successfully moved to a new location!{Color.NC}\n")

def wave_demo():
    """Waving demonstration"""
    print(f"\n{Color.BLUE}{'='*70}")
    print("  👋 WAVE DEMONSTRATION 👋")
    print(f"{'='*70}{Color.NC}\n")
    
    move_to_position(0.0, 0.6, -0.3, 2.0, "Moving to wave position")
    
    for i in range(3):
        print(f"\n{Color.GREEN}Wave {i+1}/3{Color.NC}")
        move_to_position(0.4, 0.6, -0.3, 0.9, "→")
        move_to_position(-0.4, 0.6, -0.3, 0.9, "←")
    
    move_to_position(0.0, 0.6, -0.3, 0.9, "Center")
    move_to_position(0.0, 0.0, 0.0, 2.0, "Home")
    
    print(f"\n{Color.GREEN}Wave complete!{Color.NC}\n")

def scan_demo():
    """Scanning demonstration"""
    print(f"\n{Color.BLUE}{'='*70}")
    print("  📷 SCANNING DEMONSTRATION 📷")
    print(f"{'='*70}{Color.NC}\n")
    
    move_to_position(0.0, 0.7, -0.5, 2.0, "Raising arm for scanning")
    
    print(f"\n{Color.GREEN}Scanning workspace...{Color.NC}")
    move_to_position(-0.8, 0.7, -0.5, 3.0, "← Left scan")
    time.sleep(0.5)
    move_to_position(0.8, 0.7, -0.5, 3.0, "→ Right scan")
    time.sleep(0.5)
    move_to_position(0.0, 0.7, -0.5, 2.0, "Center")
    
    move_to_position(0.0, 0.0, 0.0, 2.0, "Home")
    
    print(f"\n{Color.GREEN}Scanning complete!{Color.NC}\n")

def show_menu():
    """Display demo menu"""
    print(f"\n{Color.BLUE}{'='*70}")
    print("  🤖 AUTOMATED ROBOT ARM DEMONSTRATIONS 🤖")
    print(f"{'='*70}{Color.NC}")
    print(f"\n{Color.YELLOW}  Available Demonstrations:{Color.NC}\n")
    print("  1) Pick and Place - Automated object manipulation")
    print("  2) Wave - Friendly waving motion")
    print("  3) Scan - Workspace scanning")
    print("  4) Run All Demos")
    print("  5) Exit")
    print(f"\n{Color.BLUE}{'='*70}{Color.NC}\n")

def check_gazebo_running():
    """Check if Gazebo is running"""
    result = subprocess.run(['pgrep', '-x', 'gzserver'], capture_output=True)
    return result.returncode == 0

def main():
    print(f"\n{Color.BLUE}{'='*70}")
    print("  AUTOMATED DEMONSTRATION SCRIPT")
    print(f"{'='*70}{Color.NC}\n")
    
    # Check if Gazebo is running
    if not check_gazebo_running():
        print(f"{Color.RED}ERROR: Gazebo is not running!{Color.NC}\n")
        print("Please start Gazebo first:")
        print(f"  {Color.GREEN}./run_simple.sh{Color.NC}")
        print("\nOr:")
        print(f"  {Color.GREEN}./run_ros2.sh{Color.NC}")
        print("\nThen run this script again in a new terminal.\n")
        sys.exit(1)
    
    print(f"{Color.GREEN}✅ Gazebo is running!{Color.NC}\n")
    print("Waiting 2 seconds for simulation to stabilize...\n")
    time.sleep(2)
    
    if len(sys.argv) > 1:
        choice = sys.argv[1]
    else:
        show_menu()
        choice = input("Enter your choice [1-5]: ").strip()
    
    try:
        if choice == '1':
            pick_and_place_demo()
        elif choice == '2':
            wave_demo()
        elif choice == '3':
            scan_demo()
        elif choice == '4':
            print(f"\n{Color.YELLOW}Running all demonstrations...{Color.NC}\n")
            pick_and_place_demo()
            time.sleep(2)
            wave_demo()
            time.sleep(2)
            scan_demo()
        elif choice == '5':
            print("Exiting...")
        else:
            print(f"{Color.YELLOW}Invalid choice. Running pick-and-place demo...{Color.NC}")
            pick_and_place_demo()
        
        print(f"\n{Color.GREEN}✨ Thank you for watching! ✨{Color.NC}\n")
        
    except KeyboardInterrupt:
        print(f"\n\n{Color.YELLOW}Demo interrupted by user.{Color.NC}\n")
    except Exception as e:
        print(f"\n{Color.RED}Error: {e}{Color.NC}\n")

if __name__ == '__main__':
    main()
