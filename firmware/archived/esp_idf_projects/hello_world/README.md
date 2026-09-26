# Readme

# Initial set up of ESP-IDF

```
git clone
git submodule update --init --recursive
```

# Creating a new project using ESP-IDF

```bash
get_idf
idf.py create-project -p . <project_name>
idf.py set-target esp32-s3
idf.py menuconfig
```

# How to build this project

```bash
get_idf
idf.py build
idf.py flash
idf.py fullclean
```