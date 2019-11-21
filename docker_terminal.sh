
time_now=$(date +%s)
echo $time_now

docker run -it -v //firmware:/root/firmware -e 'TZ=America/Los_Angeles' auvic_firmware bash
