import os

def menu():
    done = False

    while done == False:
        os.system("figlet -t SARAX")
        print("What would you like to run?")
        print("1. Simulator")
        print("2. Sarax Framework")
        print("3. Exit")

        choice = int(input("\nChoice (1 - 3): "))

        if choice == 1:
            os.system(". $SARAX_WS/PX4-Autopilot/sarax_plus_sitl.bash")
        elif choice == 2:
            os.system("roslaunch m4e_mani_base sarax_plus_sitl.launch")
        elif choice == 3:
            done = True
        else:
            print("Invalid input")

if __name__ == "__main__":
    menu()