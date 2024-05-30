CORDIC IP core with AXI stream interface

# Build docker image
```sh
docker build -t ghcr.io/zaporozhets/cordic_sin_cos_axi_stream:latest .
```

# Run image localy
```sh
docker run -it --net=host -v "$(pwd)":"$(pwd)":rw --workdir "$(pwd)" ghcr.io/zaporozhets/cordic_sin_cos_axi_stream:latest bash
```

# Run tests for baseband
```sh
pytest -s ./test/
```
