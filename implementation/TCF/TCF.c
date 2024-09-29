#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>

#include "../project_config.h"

#define BUFSIZE 900

//altera a temperatura para um valor "mais certo"
typedef struct {
    double Kp;
    double Ki;
    double Kd;
    double set_point;
    double integral;
    double previous_error;
} PID;

void PID_init(PID *pid, double Kp, double Ki, double Kd, double set_point) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->set_point = set_point;
    pid->integral = 0.0;
    pid->previous_error = 0.0;
}

double PID_update(PID *pid, double current_temp) {
    double error = pid->set_point - current_temp;
    pid->integral += error;
    double derivative = error - pid->previous_error;
    double output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
    pid->previous_error = error;
    return output;
}

typedef struct {
    float temps[4];
    int heaters[4];
} Data;

void send_to_TSL(char buffer[]){ //saber qual o formato de string do buffer para enviar
    int pipe = open(RESPONSE_PIPE, O_WRONLY);
    write(pipe, buffer, strlen(buffer));
}

void control_heaters(Data *data, PID *pid){
    int size = sizeof(data->temps) / sizeof(data->temps[0]);

    for(int i = 0; i < size; i++){
        double PID_output = PID_update(pid, (double) data->temps[i]);
        printf("%f\n",PID_output);
        if(PID_output > 0.0){
            //ligar heater
            data->heaters[i] = 1;
        }
        else{
            //desligar heater
            data->heaters[i] = 0;
        }
    }
}

Data parse_input(char buffer[]){
    //Usa-se data.temps quando data não é pointer
    int index;
    Data data;
    sscanf(buffer,"%d;%f-%d;%f-%d;%f-%d;%f-%d",&index,
              &data.temps[0],&data.heaters[0],
              &data.temps[1],&data.heaters[1],
              &data.temps[2],&data.heaters[2],
              &data.temps[3],&data.heaters[3]);
    return data;
}

int main() {

    PID pid;
    PID_init(&pid, 1.0, 0.1, 0.05, 0.0); //set_point = temperatura alvo

    //fdesc[0] -> read
    //fdesc[1] -> write
    int fdesc[2]; //fdesc -> file descriptors

    //criar file descriptors para a pipe
    if(pipe(fdesc) == -1){
        perror("Error creating pipe");
        return 1;
    }

    int pipe = open(TEMP_INFO_PIPE, O_RDONLY);
    char buffer[BUFSIZE];
    while(1) {
        ssize_t bytes_read = read(pipe, buffer, sizeof(buffer));
        buffer[bytes_read] = '\0';

        printf("%s\n", buffer);

        Data data = parse_input(buffer);

        control_heaters(&data, &pid);
        int size = sizeof(data.temps) / sizeof(data.temps[0]);

        for (int i = 0; i < size; i++) {
            printf("Temps: %f\n", data.temps[i]);
            printf("Heaters: %d\n", data.heaters[i]);
        }

        //Inserir output no buffer
        char buffer_output[7];
        sprintf(buffer_output, "%d;%d;%d;%d", data.heaters[0], data.heaters[1], data.heaters[2], data.heaters[3]);
        printf("Buffer output: %s\n", buffer_output);

        send_to_TSL(buffer_output);
    }
}