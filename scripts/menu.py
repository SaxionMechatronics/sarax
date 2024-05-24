import os
import sys
from subprocess import run, Popen

TERMINAL = 'gnome-terminal'

def open_terminal(command, terminal=TERMINAL):
    if terminal == 'gnome-terminal':
        process = Popen(['gnome-terminal', '--', 'bash', '-c', f'{command}'])
    elif terminal == 'xterm':
        process = Popen(['xterm', '-e', f'bash -c "{command}"'])
    elif terminal == 'konsole':
        process = Popen(['konsole', '-e', f'bash -c "{command}'])
    elif terminal == 'xfce4-terminal':
        process = Popen(['xfce4-terminal', '-e', f'bash -c "{command}'])
    else:
        raise Exception("No compatible terminal found")

def sarax_config(system:str):
    done = False

    while done == False:
        print("\nSelect an option:")
        print("1. Install Sarax")
        print("2. Run Sarax")
        print("3. Exit")

        choice = int(input("\nOption (1 - 3): "))

        if choice == 1:
            install_sarax(system)
        elif choice == 2:
            run_sarax()
        elif choice == 3:
            done = True
        else:
            print("Invalid input")

def docker_config(system:str):
    done = False
    
    while done == False:
        print("\nSelect an option:")
        print("1. Install Docker")
        print("2. Install Docker container toolkit (GPU accelrated container)")
        print("3. Exit")

        choice = int(input("\nOption (1 - 3): "))

        if choice == 1:
            install_docker(system)
        elif choice == 2:
            install_docker_deps()
        elif choice == 3:
            done = True
        else:
            print("Invalid input")

def install_docker_deps():
    done = False

    while done == False:
        docker_deps = input("\nWould you like to install the container dependencies? (y or n): ")

        if docker_deps == 'y':
            run(["chmod +x scripts/install_docker_prerequisites.sh"], shell=True)
            run(["scripts/install_docker_prerequisites.sh"], shell=True)
        elif docker_deps == 'n':
            done = True
        else:
            print("Invalid input")

def install_docker(system:str):
    done = False

    while done == False:
        if system == "Linux":
            docker_install = input("\nWould you like to install Docker? (y or n): ")

            if docker_install == 'y':
                print("\nInstalling Docker...")
                os.system("sudo apt-get update")
                os.system("sudo apt-get install -y docker.io")
                os.system("sudo systemctl start docker")
                os.system("sudo systemctl enable docker")
                print("Docker installed successfully.")

            elif docker_install == 'n':
                done = True
            else:
                print("Invalid input")
            
        if system == "Windows (WSL2)":
            docker_install = input("\nWould you like to install Docker? (y or n): ")

            print("\nIt is advised to follow the instructions from the official Docker documentation")
            print("Official Docker Documentation: https://docs.docker.com/desktop/install/windows-install/")
            print("\nPlease install Docker Desktop and enable WSL2 backend to continue with WSL2.")
            print("Refer to the this document: https://github.com/Arief-AK/sarax/blob/main/Docs/Docker%20prerequisites.md")
            done = True

def install_sarax(system:str):
    print("This will install the dependencies and sarax source onto the machine.")
    choice = input("Do you want to continue? (y or n): ")

    while (choice != 'n'):
        try:
            if choice == 'y':
                print("\nInstalling Sarax Framework...")
                if system == "Linux":
                    print("Running commands for Linux system")
                    run(["chmod +x scripts/install_sarax_linux.sh"], shell=True)
                    run(["scripts/install_sarax_linux.sh"], shell=True)
                if system == "Windows (WSL2)":
                    print("Running commands for Windows (WSL2) backend system")
                    run(["chmod +x scripts/install_sarax_wsl2.sh"], shell=True)
                    run(["scripts/install_sarax_wsl2.sh"], shell=True)
                
                print("Sarax Framework installed successfully.")
                choice = 'n'
        
        except Exception as e:
            print(f"Exception: {e}")

def run_sarax():
    print("\nMake sure to run QGroundControl before running Sarax.")
    choice = input("Do you want to continue? (y or n): ")
    
    while choice == 'y':
        # Set executable permission
        run(["chmod +x scripts/run_sarax.sh"], shell=True)
        
        # Print menu options
        print("\nOptions:")
        print("1. Simulator\n2. Sarax Framework\n3. Exit")
        option = int(input("\nWhat would you like to run? (1 - 3): "))
        
        if option == 1:
            print("\nRunning Gazebo simulator")
            SARAX_COMMAND = "cd ~/sarax_ws/PX4-Autopilot && ./sarax_plus_sitl.bash"
            open_terminal(SARAX_COMMAND)
            print("Successfully run PX4 simulator")
        elif option == 2:
            print("\nRunning Sarax framework...")
            ROSLAUNCH_COMMAND = "echo 'ROS DISTRO: \$ROS_DISTRO' && cd ~/sarax_ws && source devel/setup.bash && roslaunch m4e_mani_base sarax_plus_sitl.launch"
            open_terminal(ROSLAUNCH_COMMAND)
            print("Successfully ran Sarax framework")
        elif option == 3:
            choice = 'n'

    choice = input("\nEnter any value to continue")

def display_menu(system:str):
    os.system("figlet -t SARAX")
    print(f"OS: {system}")
    print("What would you like to configure?")
    print("1. Sarax")
    print("2. Docker")
    print("3. Exit")

def get_OS() -> str:
    system = "Linux"
    while True:
        print("What system are you running?")
        print("1. Linux")
        print("2. Windows (WSL2)")
        choice = int(input("Enter your choice (1-2): "))

        if choice == 1:
            break
        elif choice == 2:
            system = "Windows (WSL2)"
            break
        else:
            print("Invalid choice. Please enter a number between 1 and .")
    
    return system

def main():
    system = get_OS()
    while True:
        display_menu(system)
        choice = input("Enter your choice (1-3): ")
        
        if choice == '1':
            sarax_config(system)
        elif choice == '2':
            docker_config(system)
        elif choice == '3':
            print("Exiting...")
            sys.exit()
        else:
            print("Invalid choice. Please enter a number between 1 and 4.")

if __name__ == "__main__":
    main()
