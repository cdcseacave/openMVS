
# Building & running openMVS using Docker

## Quick start:

1. Make sure docker is installed on your local machine.
2. Run the 'easy start' script, using the *full local path* to the folder with your SFM input files (perhaps output from openMVG or COLMAP):

	./QUICK_START.sh /path/where/your/SFM/results/are

3. This will put you in a directory (inside the Docker container) mounted to the local path you specified so that you can run openMVS binaries on your own SFM inputs. Enjoy!

## Build from scratch:

You can also build the docker image from scratch based on the **Dockerfile** (perhaps with your own changes / modifications) using:

        ./buildFromScratch.sh /path/where/your/SFM/results/are

## NOTES

+ This workflow is pinned to build from [openMVS 1.0](https://github.com/cdcseacave/openMVS/releases/tag/v1.0). To build from a different release, or build from the latest commit on master, open up the Dockerfile and comment/uncomment as directed.

+ Running openMVS binaries can use a lot of memory (depending on the size of your data set/ imagery). Docker has a relatively small default memory setting (2Gb on Mac). You will probably want to increase this before you run any larger workflows. From Docker desktop on Mac for example, just open the Docker GUI, go to the *Advanced* tab and increase via the slider:

![alt text][dockerParam]

[dockerParam]: https://i.stack.imgur.com/6iWiW.png "Recommend increasing memory to >4Gb"
