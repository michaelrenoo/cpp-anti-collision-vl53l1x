
import os

folders = []
c_files = []

root = os.path.dirname(os.path.realpath(__file__)) + "/../main/"
os.chdir(root)

build_dir = "../build"

ignore_dirs = ["driver_esp32", "Example", "rapidjson", "tests"]
ignore_files = ["digest.cpp", "DataTypeExample.cpp"]

include_dirs = ["."]  # , "libs/rapidjson/include"]
include_dirs_arg = "-I " + " -I ".join(include_dirs)


def check_folder(root_path):
    path = root_path
    if len(path) <= 0:
        path = "."
    for item in os.listdir(path):
        # if not "esp32" in item and not "Example" in item and not "test" in item:
        # if not any([x in item for x in ignore_dirs]):
        if os.path.isfile(path + '/' + item):
            if ".c" in item:
                if len(path) <= 0:
                    c_files.append(item)
                else:
                    if item not in ignore_files:
                        c_files.append(path + '/' + item)
        else:
            if not item in ignore_dirs:
                if len(path) <= 0:
                    folders.append(item)
                    check_folder(item)
                else:
                    folders.append(path+'/'+item)
                    check_folder(path+'/'+item)


if __name__ == "__main__":

    ################################
    # create sh file

    f = open("../build_pi3.sh", "w")
    f.write("#! /bin/bash\n")
    f.write("cd main\n")
    f.write("make -j4 -f makefile_pi3 $1\n")
    f.close()

    ################################
    # analyze file structure

    check_folder("")
    c_files = [x.replace("./", "") for x in c_files]
    folders = [x.replace("./", "") for x in folders]
    o_files = [x.replace(".cpp", ".o").replace(".c", ".o") for x in c_files]

    ################################
    # create makefile

    f = open("makefile_pi3", "w")

    ################################
    # main rule

    f.write("main: create_files_tructure gitversion.hpp {}/{}\n".format(build_dir,
                                                                        (" {}/".format(build_dir)).join(o_files)))
    f.write("\tg++ -o {}/main {:s} -g {}/{} -lpthread -lm -lwiringPi -li2c\n".format(build_dir,
                                                                               include_dirs_arg, build_dir, (" {}/".format(build_dir)).join(o_files)))
    f.write("\n")

    ################################
    # create folder structure in build forlder

    f.write("create_files_tructure:\n")
    f.write(
        "\tmkdir -p {}\n".format(build_dir))
    for i in range(0, len(folders)):
        dir_path = "{}/{}".format(build_dir, folders[i])
        f.write(
            "\tmkdir -p {}\n".format(dir_path))

    # copy python scripts
    f.write("\tmkdir -p {}/scripts\n".format(build_dir))
    f.write("\tcp driver/driver_pi/scripts/* {}/scripts\n".format(build_dir))
    # f.write("\tcp scripts/* {}/scripts\n".format(build_dir))
    f.write("\n")

    ################################
    # generator for gitverison.hpp

    f.write("gitversion.hpp:\n")
    f.write('\techo "#ifndef GITVERSION_H_\\n#define GITVERSION_H_\\n#define GIT_HASH 0x$(shell git rev-parse --short HEAD)UL\\n#define GIT_BRANCH \\"$(shell git rev-parse --abbrev-ref HEAD)\\"\\n#endif\\n" > $@\n')
    f.write("\n")

    ################################
    # write make rules

    for i in range(0, len(c_files)):
        f.write("{}/{}: {}\n".format(build_dir, o_files[i], c_files[i]))
        f.write("\tg++ -c {:s} -g {} -o {}/{}\n".format(include_dirs_arg,
                                                        c_files[i], build_dir, o_files[i]))
        f.write("\n")

    f.write("\n")
    f.write("clean:\n")
    # f.write("\tfind . -maxdepth 5 -type f -name \"*.o\" -delete\n")
    f.write("\trm -f -r {}\n".format(build_dir))
    f.write("\trm -f gitversion.hpp\n")
    f.write("\techo Clean done\n")
    f.close()
    print("done")
