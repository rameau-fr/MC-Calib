# Building docker images

  - [Install](https://docs.docker.com/engine/install/) docker

  - Create the image:
   
      ```bash
      docker build --target prod -t mc-calib-prod . # production environment
      docker build --target dev -t mc-calib-dev .   # development environment
      ```

  - Build and push to DockerHub:

    ```bash
    docker buildx create --use

    docker buildx build --platform linux/amd64,linux/arm64 --target prod --push -t bailool/mc-calib-prod .

    docker buildx build --platform linux/amd64,linux/arm64 --target dev --push -t bailool/mc-calib-dev .
    ```