# ArduPilot tasks

Como su predecesor (ardupilot), BeaglePilot utiliza la siguiente estructura para programar tareas en el piloto automático (el siguiente código se puede encontrar en `ArduCopter/ArduCopter.pde`, `ArduPlane/ArduPlane.pde` y `APMRover2/APMRover2.pde`). Esto se hace de esta manera para que el sistema operativo que ejectura lleve a cabo estas latencias.

```
static const AP_Scheduler::Task scheduler_tasks[] PROGMEM = {
...
    { gcs_send_heartbeat, 100, 150 },
    { update_notify, 2, 100 },
    { one_hz_loop, 100, 420 },
    { gcs_check_input, 2, 550 },
    { gcs_send_heartbeat, 100, 150,
    { gcs_send_deferred, 2, 720 },
    { gcs_data_stream_send, 2, 950 },
...
}
```
- El *primer parámetro* es el nombre de la función
- El *segundo* es el *tiempo que debería tardar* en unidades de 10ms (por ejemplo: 2 significa 20ms que produce 50Hz, por lo tanto, esta función ejecuta 50 veces por segundo).
- El *tercer parámetro* es el *tiempo máximo más allá del cual la función no debe ejecutar*.

