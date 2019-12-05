
# Building & running openMVS using Docker

You may want to build and run openMVS in Docker instead of on your local machine. To do so, follow these steps:

1. Make sure docker is installed on your local machine.
2. Run the included script, using the *full local path* to the folder with your SFM input files (perhaps output from openMVG or COLMAP):

	./runMe.sh /path/where/your/SFM/results/are

3. This will put you in a directory (inside the Docker container) mounted to the local path you specified so that you can run openMVS binaries on your own SFM inputs. Enjoy!


##NOTE 
This workflow is pinned to build from [openMVS 1.0](https://github.com/cdcseacave/openMVS/releases/tag/v1.0). To build from a different release, or build from the latest commit on master, open up the Dockerfile and comment/uncomment as directed.