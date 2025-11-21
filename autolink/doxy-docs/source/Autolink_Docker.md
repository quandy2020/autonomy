# How to Develop Autolink inside Docker on Both x86_64 and ARM64

To make life easier, Apollo maintains Autolink Docker images and a number of
scripts to help developers build and play with Autolink framework.

Autolink Docker images are built upon Ubuntu 18.04, and comes with full support for
Autolink framework. For those who are interested in Autolink only, this would be
the ideal point to start with.

**Note: Apollo team also maintains ARM64 Autolink Docker images which was tested on
Jetson AGX Xavier. The same set of scripts are provided to run both on x86_64
and ARM64 platforms.**

In the next section, we will show you how to play with Autolink Docker images.

## Build and Test Autolink inside Docker

Run the following command to start Autolink Docker container:

```bash
bash docker/scripts/autolink_start.sh
```

Or if you are in China, run:

```bash
bash docker/scripts/autolink_start.sh -g cn
```

A Docker container named `apollo_autolink_$USER` will be started.

**Note**: You will lose all your previous changes in the container if you have
ran this command before. Unless you would like to start a fresh Docker
environment.

To log into the newly started Autolink container:

```bash
bash docker/scripts/autolink_into.sh
```

**Note**: you can login and logout Autolink container multiple times as you wish,
the Docker environment will stay there until the next time you start another
Autolink container.

To build Autolink only and run unit tests:

```bash
./apollo.sh build autolink
./apollo.sh test autolink
```

Or run with `--config=opt` for an optimized build/test.

```bash
./apollo.sh build --config=opt autolink
./apollo.sh test --config=opt autolink
```

You should be able to see that all the testcases passed.
