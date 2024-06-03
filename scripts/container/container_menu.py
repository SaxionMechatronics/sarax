import os

def system_init() -> str:
    done = False
    os_sys = "Linux"

    while done == False:
        os.system("figlet -t SARAX")
        print("What host system are you running?")
        print("1. Linux")
        print("2. Windows WSL2")
        
        choice = int(input("\nChoice (1 - 2): "))

        if choice == 1:
            done = True
            print(f"Initialised for {os_sys} host system")
        elif choice == 2:
            os_sys = "WSL2"
            os.system("cd ~ && sed -i 's/^# export LD_LIBRARY_PATH=\/usr\/lib\/wsl\/lib$/export LD_LIBRARY_PATH=\/usr\/lib\/wsl\/lib/' .bashrc")
            os.system("cd ~ && sed -i 's/^# export LIBVA_DRIVER_NAME=d3d12$/export LIBVA_DRIVER_NAME=d3d12/' .bashrc")
            print(f"Initialised for {os_sys} host system")
            done = True
        else:
            print("Invalid input")

    return os_sys

def menu(os_sys) -> int:
    done = False

    while done == False:
        os.system("figlet -t SARAX")
        print(f"HOST SYSTEM: {os_sys}")
        print("What would you like to run?")
        print("1. Simulator")
        print("2. Sarax Framework")
        print("3. Exit")

        choice = int(input("\nChoice (1 - 3): "))

        if choice == 1:
            os.system("cd $SARAX_WS/PX4-Autopilot && ./sarax_plus_sitl.bash")
        elif choice == 2:
            os.system("roslaunch m4e_mani_base sarax_plus_sitl.launch")
        elif choice == 3:
            done = True
        else:
            print("Invalid input")
    
    return choice

if __name__ == "__main__":
    os_sys = system_init()
    menu(os_sys)