EXE = main
SRC = main.c ./CAN/can.c ./MPI204A/MPI204A.c ./Timer/Timer.c ./PGV150/PGV150.c ./UART/UART.c ./Odom_Calib/Odom_Calib.c ./Math/Trigonometric.c ./App/AGVControl.c ./App/Interpolation.c ./App/Movement.c ./App/motion_ctrl.c ./Thread_Pool/Thread_Pool.c ./TCP/TCP.c ./TCP/1200.c ./AGV/agv.c ./SPC_SDIO_1616/SPC_SDIO_1616.c ./Tools/Tools.c
HEADER_PATH = -I"/home/siasunhebo/siasun_xieqizhen/AGV/Test-feature-B" -I./sensorTest/ -I./TCP/ -I./App -I./CAN/ -I./Math/ -I./Odom_Calib/ -I./MPI204A -I./Timer -I./UART -I./PGV150 -I./AGV -I./include -I./Toolsb -I./Thread_Pool -I./SPC_SDIO_1616

$(EXE):
	$(CC) $(HEADER_PATH) $(SRC) -o $(EXE) -g -lpthread -lrt -lm


clean:
	rm -rf main *.o
