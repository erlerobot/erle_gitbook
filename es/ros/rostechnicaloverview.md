# Resumen técnico

Esta visión general técnica entra en más detalles acerca de la implementación de ROS. La mayoria de los usuarios de ROS no necesiran conocer estos detalles, pero son importantes para aquellos que deseen escribir sus propias bibliotecas de un cliente ROS o aquelles que deseean integrar otros sistemas con ROS.

Esta visión general sume que ya esta familiarizado con el sistema ROS y sus concpetos. Por ejemplo, el panorama conceptual de ROS proporcionado un vistazo general sobre su arquitectura computacional en forma de grafo, incluyendo el rol del Master y los nodos de ROS.

###Master

El Master se implementa a través de `XMLRPC`, que es un *protocolo HTTP sin estado*. `XMLRPC` fuer elegido principalmente porque es relativamente ligero, no requiere el estado de la conexión, y está disponible en un gran variedad de lenguajes de programación. Por ejemplo, en Python puedes iniciar un interprete y comenzar a interactuar con el ROS Master.

``` python
$ python
>>> from xmlrpclib import ServerProxy
>>> import os
>>> master = ServerProxy(os.environ['ROS_MASTER_URI'])
>>> master.getSystemState('/')
[1, 'current system state', [[['/rosout_agg', ['/rosout']]], [['/time', ['/rosout']], ['/rosout', ['/rosout']], ['/clock', ['/rosout']]], [['/rosout/set_logger_level', ['/rosout']], ['/rosout/get_loggers', ['/rosout']]]]]
```

El Master tiene una **API de registro**, que permite a los nodos registrarse como publicadores, suscriptores o proveedores de servicios. El Máster tiene un RUI y alamacena en la variable de entorno `ROS_MASTER_URI`. Esta URI corresponde al hos: puerto del servicio XML_RPC que se está ejecutando. Por defecto, **el Master inicio en el puerto 11311**

Para más información, por favor consulte [MASTER API](http://wiki.ros.org/ROS/Master_API).

### Parameter Server
Aunque [Parameter Server](http://wiki.ros.org/Parameter%20Server) es en realidad una parte de ROS [Master](http://wiki.ros.org/Master).

Al igual que el API Master, el API de *Parameter Server* es implementado a través de `XMLRPC`. El uso de `XMLRPC` permite una integración con sencilla con la libreria de clientes de ROS y porporciona una mayor flexibilidad  en cuento a almacenamiento y recuperación de datos. El *Parameter Serve* puede almacenar `basic XML-RPC scalars` (`32-bit integers`, `booleans`, `strings`, `doubles`, `iso8601 dates`), `lists`, and `base64-encoded binary data`. El *Parameter Server* *can also store dictionaries* (por ejemplo structs), pero estas tienen un significado especial.

El *Parameter Server* **utiliza una repesentación de diccionario de diccionario** para los *nombres de espacio (namespaces)*, donde cada diccionario representa un nivel en la jerarquúi de nombrado. Esto significada que *cada clave del diccionario representa un namespace*. Si el valor es un diccionario, el *Parameter Server* asume que se esta almacenando el nombre de un *namespace*. Por ejemplo, si se establece el parámetro `/ns1/ns2/foo` al valor 1, el valor de `/ns1/ns2/` sería un diccionario `{foo : 1}`  y el valor de `/ns1` sería un diccionario `{ns2 : { foo : 1 }}`.

El `API XMLRPC` hace que sea muy fácil de integrar llamadas al *Parameter Server* sin tener que usar la biblioteca de cliente de ROS. Asumiendo que usted tiene acceso a una biblioteca de un cliente `XMLRPC`, se pueden hacer llamadas directamente.Por ejemplo:

``` python
$ python
>>> from xmlrpclib import ServerProxy
>>> import os
>>> ps = ServerProxy(os.environ['ROS_MASTER_URI'])
>>> ps.getParam('/', '/foo')
[-1, 'Parameter [/bar] is not set', 0]
>>> ps.setParam('/', '/foo', 'value')
[1, 'parameter /foo set', 0]
>>> ps.getParam('/', '/foo')
[1, 'Parameter [/foo]', 'value']
```

Por favor, revise [Parameter Server API](http://wiki.ros.org/ROS/Parameter%20Server%20API) para ver más detalles sobre este API.

###Nodos
Los nodos de ROS tiene varias APIs:

1. Un API esclava. Es un API XMLRPC que tiene dos funciones: *recibir callbacks del Mater*, y *negociar conexiones con otros nodos*. Para más detalles sobre esta API consulta [Slave API](http://wiki.ros.org/ROS/Slave_API).
2. Una implementación de un proctocolo de transporte de *topic* (ver [TCPROS](http://wiki.ros.org/ROS/TCPROS) y [UDPROS](http://wiki.ros.org/ROS/UDPROS)). Lo nodos establecen conexiones de *topic* entre sí utilizando el protocolo acordado. El protocolo más general es TCPROS, que utiliza conexciones de TCP/IP persistentes con estado.
3. Una API de línea de comandos. Todos los nodos deben soportar [command-line remapping arguments](http://wiki.ros.org/Remapping%20Arguments), que permite configurar los nombres de los nodos en tiempo de ejecución.


Cada nodo tiene un URI, que corresponde al host: El puerte del [servidor XMLRPC ](http://wiki.ros.org/ROS/Master_Slave_APIs) esta ejecutando. El servidor XMLRPC no se utliiza transporte de *topic* o servicio de datos: en su lugar, se utiliza para negociar conexiones con otros nodos y también para comunicarse con el maestro. Este servidor es creado y gestionado dentro de la biblioteca cliente de ROS, pero es general no es visible para el usuario de la biblioteca cliente de ROS. El servidor XMLRPC puede estar unido a cualquier puerto en el host donde se ejecuta el nodo.

El servidor XMLRPC proporciona una [Slave API](http://wiki.ros.org/ROS/Slave_API), que permite que el nodo recibir *actualizaciones de publicación* llamadas desde el Master. Estas actualizaciones contieen un nombre de *topic* y una lista de URISs para los nodos que publican ese *topic*. El servidor XMLRPC también recibirá las llamadas de los suscriptores que están esperando una respuesta de la conexión de *topic*. En general, cuando un nodo recibe una actualización de publicación, se conectará a los nuevos publicadores.

El transporde de *topic* se negocia cuando el suscriptor solocita la conexión de un *topic* utilizando un servirdo de publicaciones XMLRPC. El suscriptor encía al publicardor una lista de protocolos soportados. El publicador seleciona un protocolo de la lsita, como [TCPROS](http://wiki.ros.org/ROS/TCPROS), y devuelve la configuración necesaria para el protocolo (por ejemplo, una IP y un puerto para una conexión TCP). El suscriptor que establece una conexión separada utilizando la configuración suministrada.

###Transporte de *Topic*
Hay muchas maneras de enviar información por la red, cada una tienes sus ventajes e desventajas, dependiendo en gran medida de la aplicación. TCP es ampliamente utilizado, ya que proporciona un canal de comunicación simple y fiable. Los paquetes TPC siempre llegan en orden, y los paquete se reenvían hasta que llegan. Mientas que para redes cableadas Ethernet funciona bien, esto no es tan bueno con utilizado una red Wifi con grandes perdidas o una conexión móvil, UDP es más apropiado. Cuando verias suscriptores se agrupan en una sola subred puede ser más eficiente para el publicador comunicarse con todos ellos utilizando difusión UDP (broadcast UDP).


Por estas razones, ROS no se compromete solo con un protocolo. Dado un editor de URI, un nodo suscriptor negocia su conexión, utilizando el transporte adecuado, con ese publicador a través de XMLRPC. El resultado de la negociación es que los dos nodos esán conectados, con un flujo de mensajes desde el publicador al suscriptor.


Cada transporte tiene su propio protocolo de como se intercambia los datos del mensajes.
Por ejemplo, cuando utizados TCP, la negociación involucraría al publicador darla al suscriptor la dirección IP y el puerto en esa llamada de conexión. El suscriptor crearía una socket TCP/IP a la dirección y puerto especificado. Los nodos intercambian una [cabecera de conexión](http://wiki.ros.org/ROS/Connection%20Header) que incluye información como la suma MD5 del tipo de mensaje y el nombre del *topic*, y además el publicador comienza a enviar los datos del mensaje serializados directamente sobre el socket.

Cabe destacar, los nodos se comunican directamente entre sí, sobre un mecanismo de transporte adecuado. Los daos no se encaminan a través del Master. Los datos no se envían a través de XMLRPC. El sistema XMLRPC solo se utiliza para negociar la conexión de datos.

Developer links:

- [ROS/TCPROS](http://wiki.ros.org/ROS/TCPROS)
- [ROS/UDPROS](http://wiki.ros.org/ROS/UDPROS)

###Serialización de mensaes y suma MD5 de mensajes
Los mensajes son serializados en un representación muy compacta que corresponde aproximadamente a una serialización de una estructura de C de los datos del mensaje en el formato little endian. La representación compacta significa que los dos nodos de comunicación deben ponerse de acuerdo sobre la distribución de los datos en el mensaje.

Los tipos de mensajes ([msgs](http://wiki.ros.org/msg)) en ROS están versionados utilizando un suma especial MD5 del mensaje. En general la librería cliente no implementa el calculo de la suma MD5 directamente, en lugar de guardar esta suma MD5  en el código fuente del mensaje utilizando la salida `roslib/scripts/gendeps`. A modo de referencia, la suma MD5 se calculada a partir del texto MD5 del fichero `msg`:

- contenido eliminadocomments removed
- espacios en blanco elimiandos
- nombre de paquetes y dependencias elimiandas
- constantes reordenadas al inicio por delante de otras declaraciones

Con el fin de captar cambios que se producen en los tipos de mensaje, el tecto MD5 se concatena con el texto MD5 de cada uno de los tipos implícitos en el orden que aparecen.

###Estableciendo la conexión con un *topic*

Poniendo todo junto, la secuencia en la que los dos nodos comienzan a intercambiar los mensajes es:

1. Comienza el suscriptor. Lee los argumentos de la línea de comandos para resolver el nombre del *topic* que se va a utilizar.
2. Comienza el publicador.Lee los argumentos de la línea de comandos para resolver el nombre del *topic* que se va a utilizar.
3. El suscriptor se registra en el Master. (XMLRPC)
4. El publicador se registra en el Master. (XMLRPC)
5. El Master indica al suscriptor de un nuevo publicador. (XMLRPC)
6. El suscriptor contacto con el publicador para solicitar un conexión de *topic* y negocia el protocolo de transporte. (XMLRPC)
7. El publicador envía al suscriptor la configuración del protocolo de transporte selecionado. (XMLRPC)
8. El suscriptor se conecta al publicador usando el protocolo de transporte selecinado. (TCPROS, etc...)

La tramo de XMLRPC:

```
/subscriber_node → master.registerSubscriber(/subscriber_node, /example_topic, std_msgs/String, http://hostname:1234)
```
El Master devulve que no hay publicadores activos.
```
/publisher_node → master.registerPublisher(/publisher_node, /example_topic, std_msgs/String, http://hostname:5678)
```
El Master se da cuenta de que  `/subscriber_node` esta interesado en `/example_topic`,  por lo que emite un callback al suscriptor
```
master → subscriber.publisherUpdate(/publisher_node, /example_topic, [http://hostname:5678])
```

El suscriptor recibe esto y no esta conectado a `http://hostname:5678` todavía, por lo que contacta con para soliciar el *topic*.
```
subscriber → publisher.requestTopic(/subscriber_node, /example_topic, [[TCPROS]])
```
El publicador devuelve TCPROS como protocolo., por lo que el suscriptor crea una nueva conexión TCPROS con el publicador `host:port`.

###Establecíendo una conexión de servicio
No hemos hablado de los servicios mucho en esta descripción, pero se puede ver como una versión simplificado de los *topic*. Mientras que los temas puede muchos muchos publicadores, solo puede haber un solo proveedor de servicios. El nodo más reciente para registrarse con el maestro es considerado el proveedor de servicio actual. Esto permite una configuración de protocolo mucho más sencilla, de hecho, un cliente de servicio no tiene que ser un nodo de ROS.

1. El servicio se registra en el Master
2. El servicio cliente ve el servicio en el Master
3. El servicio cliente crea una conexión TCP/IP con el servicio
4. El servicio cliente y el servicio intercambia una cabecera de conexión
5. El servicio cliente envía un mensaje de petición serializada
6. El servicio responde con un mensaje de respuesta serializado

Si lo últimos pasos te resultan familiares, es porque son una extensión del protocolo TCPROS. De hecho, `rospy` y `roscpp`utilizan el mismo socket de servicio TCP/IP para recibir mensajes de conexión un *topic* y servicios.

Como no hay un callback del MASter cuando se registra un nuevo servicio, muchos bibliotecas clientes proporciona un método en la API *a la espera de servicio*. Esta simplemente pregunta constantemente hasta que un servicio se registra.