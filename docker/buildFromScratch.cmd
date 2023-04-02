@echo off

set WORKSPACE=%CD%

set CUDA_BUILD_ARGS=
set CUDA_RUNTIME_ARGS=
set CUDA_CONTAINER_SUFFIX=
set MASTER_ARGS=
set DISPLAY_ARGS=

:parse_args
if "%~1" == "" goto build_image
if "%~1" == "--cuda" (
  set "CUDA_BUILD_ARGS=--build-arg CUDA=1 --build-arg BASE_IMAGE=nvidia/cuda:11.8.0-devel-ubuntu22.04"
  set "CUDA_RUNTIME_ARGS=--gpus all -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics"
  set "CUDA_CONTAINER_SUFFIX=-cuda"
  shift
  goto parse_args
)
if "%~1" == "--master" (
  set "MASTER_ARGS=--build-arg MASTER=1"
  shift
  goto parse_args
)
if "%~1" == "--workspace" (
  set "WORKSPACE=%~2"
  shift
  shift
  goto parse_args
)
echo Unknown argument: %~1
exit /b 1

:build_image
echo Running with workspace: %WORKSPACE%

docker build -t="openmvs-ubuntu%CUDA_CONTAINER_SUFFIX%" --build-arg "USER_ID=%USERPROFILE:~-4%" --build-arg "GROUP_ID=%USERPROFILE:~-4%" %CUDA_BUILD_ARGS% %MASTER_ARGS% .
if errorlevel 1 exit /b 1

set "XSOCK=\\.\pipe\X11-unix"
set "XAUTH=%TEMP%\docker.xauth"
docker run %CUDA_RUNTIME_ARGS% --entrypoint bash --ipc=host --shm-size=4gb -w /work -v "%WORKSPACE%:/work" -e "DISPLAY=%COMPUTERNAME%:0.0" -v "%XAUTH%:%XAUTH%:rw" -e "XAUTHORITY=%XAUTH%" -it openmvs-ubuntu%CUDA_CONTAINER_SUFFIX%

exit /b 0
