import os
import sys
from subprocess import run, Popen

TERMINAL = 'gnome-terminal'
DOCKER_INSTRUCTIONS_URL = "https://github.com/Arief-AK/sarax/blob/main/Docs/Sarax%20with%20Docker.md"
DOCKER_CONTAINER_REPO = "ghcr.io/arief-ak/sarax-framework"
DOCKER_COTAINER_TAG = "latest"
DOCKER_CONTAINER_NAME = "sarax_container"

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
        os.system("figlet -t SARAX CONFIG")
        print("\nSelect an option:")
        print("1. Install Sarax")
        print("2. Run Sarax")
        print("3. Exit")

        choice = int(input("\nOption (1 - 3): "))

        if choice == 1:
            install_sarax(system)   
        elif choice == 2:
            run_sarax(system)
        elif choice == 3:
            done = True
        else:
            print("Invalid input")

def docker_config(system:str):
    done = False
    
    while done == False:
        os.system("figlet -t SARAX DOCKER CONFIG")
        print("\nSelect an option:")
        print("1. Install Docker")
        print("2. Install Docker container toolkit (GPU accelrated container)")
        print("3. Check Docker")
        print("4. List images")
        print("5. Download image")
        print("6. Run container")
        print("7. Start container (start another instance of the container)")
        print("8. Stop container")
        print("9. Remove container")
        print("0. Exit")

        choice = int(input("\nOption (0 - 9): "))

        if choice == 1:
            install_docker(system)
        elif choice == 2:
            install_docker_deps()
        elif choice == 3:
            os.system("sudo docker run hello-world")
        elif choice == 4:
            os.system(f"./docker_container.sh {DOCKER_CONTAINER_REPO}:{DOCKER_COTAINER_TAG} {DOCKER_CONTAINER_REPO} {DOCKER_COTAINER_TAG} {system} {DOCKER_CONTAINER_NAME} Images")
        elif choice == 5:
            os.system(f"./docker_container.sh {DOCKER_CONTAINER_REPO}:{DOCKER_COTAINER_TAG} {DOCKER_CONTAINER_REPO} {DOCKER_COTAINER_TAG} {system} {DOCKER_CONTAINER_NAME} Download")
        elif choice == 6:
            os.system(f"./docker_container.sh {DOCKER_CONTAINER_REPO}:{DOCKER_COTAINER_TAG} {DOCKER_CONTAINER_REPO} {DOCKER_COTAINER_TAG} {system} {DOCKER_CONTAINER_NAME} Run")
        elif choice == 7:
            os.system(f"./docker_container.sh {DOCKER_CONTAINER_REPO}:{DOCKER_COTAINER_TAG} {DOCKER_CONTAINER_REPO} {DOCKER_COTAINER_TAG} {system} {DOCKER_CONTAINER_NAME} Start")
        elif choice == 8:
            os.system(f"./docker_container.sh {DOCKER_CONTAINER_REPO}:{DOCKER_COTAINER_TAG} {DOCKER_CONTAINER_REPO} {DOCKER_COTAINER_TAG} {system} {DOCKER_CONTAINER_NAME} Stop")
        elif choice == 9:
            os.system(f"./docker_container.sh {DOCKER_CONTAINER_REPO}:{DOCKER_COTAINER_TAG} {DOCKER_CONTAINER_REPO} {DOCKER_COTAINER_TAG} {system} {DOCKER_CONTAINER_NAME} Remove")
        elif choice == 0:
            done = True
        else:
            print("Invalid input")

def install_docker_deps():
    done = False

    while done == False:
        docker_deps = input("\nWould you like to install the container dependencies? (y or n): ")

        if docker_deps == 'y':
            run(["$PWD/install_docker_prerequisites.sh"], shell=True)
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
                os.system("./install_dokcker.sh")
                print("Docker installed successfully.")

            elif docker_install == 'n':
                done = True
            else:
                print("Invalid input")
            
        if system == "WSL2":
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
                    run(["$PWD/install_sarax_linux.sh"], shell=True)
                if system == "WSL2":
                    print("Running commands for WSL2 backend system")
                    run(["$PWD/install_sarax_wsl2.sh"], shell=True)
                
                print("Sarax Framework installed successfully.")
                choice = 'n'
        
        except Exception as e:
            print(f"Exception: {e}")

def run_sarax(system:str):
    print("\nMake sure to run QGroundControl before running Sarax.")
    choice = input("Do you want to continue? (y or n): ")
    
    # Execute the Bash script
    result = run(["./check_existence.sh"], capture_output=True, text=True)
    output = int(result.stdout.strip())

    if output == 1:
        while choice == 'y':
        
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
    else:
        print("Sarax workspace is not found, please install sarax first")
        install_prompt = input("\nWould you like to install sarax? (y or n): ")

        if install_prompt == 'y':
            install_sarax(system)

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
        print("2. WSL2")
        choice = int(input("Enter your choice (1-2): "))

        if choice == 1:
            break
        elif choice == 2:
            system = "WSL2"
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
