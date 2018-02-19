# FlashAlgo

## Development Setup
Skip any step where a compatible tool already exists

1. Install [Python 2.7.9](https://www.python.org/downloads/) and make sure it's added to path
2. Install [Git](https://git-scm.com/downloads) and make sure it's added to path
3. Install [Keil MDK-ARM](https://www.keil.com/download/product/)
4. Install virtualenv in python

```
> git clone https://github.com/mbedmicro/DAPLink
> pip install virtualenv
> virtualenv venv
>
```

## Develop
1. Update tools and generate project files. This should be done everytime you pull new changes

```
> "venv/Scripts/activate"
> pip install -r requirements.txt
> progen generate -t uvision
> "venv/Scripts/deactivate"
```


```
> cd tools
> launch_uvision.bat

```
Now open the project file for the desired target in \projectfiles\uvision\<target>\

To change the RAM base address to something other than the default value of 0x20000000, add the argument  --blob_start 0x[RAM ADDRESS] in Projects...Options...User...After Build/Rebuild section of the uVision project.


## Adding a new project
For adding new targets start from template and use these docs...

## Contribute
Check out the issue tracker.

##ToDo
- Create a test
- Document how to use
- Document how to contribute
