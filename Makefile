uuv.up:
	@xhost +
	@docker start uuv
uuv.down:
	@xhost +
	@docker stop uuv
uuv.restart:
	@xhost +
	@docker restart uuv
uuv.shell:
	@xhost +	
	@docker exec -it uuv bash
uuv.build:
	@docker build -t uuv .