
time_now=$(date +%s)
echo $time_now

docker run -it -v //firmware:/root/firmware auvic_firmware bash
