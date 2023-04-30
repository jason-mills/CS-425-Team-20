#file_name = "C:/spongeBoyMeBob/test_name/5.xyz"
import sys
import tkinter as tk

from GUI import GUI


def main():

    if(len(sys.argv)) == 0:
        print("Command Line Argument Not Found. Exiting...")
        sys.exit()

    # Create Tkinter window
    root = tk.Tk()
    root.title("Open3D Visualization")

    # Create GUI object
    gui = GUI(root)

    # Run the Tkinter mainloop
    root.mainloop()


if __name__ == "__main__":
    main()
