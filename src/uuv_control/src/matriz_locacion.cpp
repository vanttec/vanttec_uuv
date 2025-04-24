// Este nodo recibe la salida de control del pid y la convierte en la accion de los motores
// https://docs.google.com/document/d/1J46TvV1WbtOj7GB3-SL5VAXwkTM1FQy_/edit?usp=sharing&ouid=118337403111481747146&rtpof=true&sd=true

// Se subscribe a:

// "uuv/control_output"

// Hay que checar que es lo que recibe la stm32. En teoria lo que ocupamos es un pwm entre 0 y 255 (sin contar que esta limitado al 50%)
// En teoria no publica nada, sino que puede enviar la senal por can a la stm32.
// Dependiendo de como funcione lo de can, sera si ocupa un nodo para enviar la senal o lo hace directamente