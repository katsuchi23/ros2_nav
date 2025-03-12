import os

current_dir = os.path.dirname(os.path.realpath(__file__)) # this is to get package directory INSIDE SRC not inside install folder (see note because when the file is runned, it doesn't runned from the src file)
current_package_dir = os.path.abspath(os.path.join(current_dir, '..'))
print(current_package_dir)

