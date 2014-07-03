ROS : Conceptos
=========

ROS tiene tres niveles de conceptos: el nivel del sistema de archivos ** **, el ** nivel de Computación Gráfica ** y ** el nivel de la comunidad . ** Estos niveles y conceptos se resumen a continuación y las secciones posteriores entran en cada una de ellas con mayor detalle .

Además de los tres niveles de conceptos , ROS también define dos tipos de nombres: ** Nombres de los recursos del paquete ** y ** Nombres Gráfico de recursos **, también se analizan a continuación .

Nivel del sistema de archivos
-----------

Los conceptos de nivel de sistema de archivos se refieren principalmente a recursos de ROS que encuentre en el disco, como por ejemplo:

- **Paquetes** : Los paquetes son la unidad principal de la organización de software de ROS . Un paquete puede contener los procesos de ROS en tiempo de ejecución (nodos) , una biblioteca ROS- dependiente , conjuntos de datos , archivos de configuración , o cualquier otra cosa que se organiza eficazmente juntos. Los paquetes son el elemento de construcción más atómica y el punto de liberación de ROS . Lo que significa que lo más granular que pueda construir y la liberación es un paquete .
- **Metapaquetes**: Metapaquetes son paquetes especializados que sólo sirven para representar un grupo de otros paquetes relacionados. Más comúnmente metapaquetes se utilizan como lugar reservado compatible para Stacks rosbuild convertidos.
- ** Manifiestos del paquete **: Manifiestos ( Paquete.xml ) proporcionan metadatos sobre un paquete, incluyendo su nombre, la versión , la descripción, información sobre la licencia , las dependencias , y otra información de meta como paquetes exportados . El manifiesto del paquete package.xml se define en el REP- 0127 .
- **Repositorios**: Una colección de paquetes que comparten un sistema VCS común. Paquetes que comparten un VCS comparten la misma versión y puede ser liberado junto con la versión amento herramienta de automatización de la floración. A menudo, estos depósitos se asignarán a Stacks rosbuild convertidos. Repositorios también pueden contener sólo un paquete.
- **(NVI) Tipos**: Descripciones de los mensajes almacenados en my_package / msg / MyMessageType.msg , definen las estructuras de datos para los mensajes enviados en ROS .
- **Servicio (SRV) tipos**: Descripciones del servicio , almacenados en my_package / srv / MyServiceType.srv , definen la solicitud y las estructuras de datos de respuesta para los servicios de ROS .

Nivel Gráfico Computación
-----------

La Computación Gráfica es la red de procesos de ROS que se están procesando los datos en conjunto de peer -to-peer . Los conceptos básicos de Computación Gráfica de ROS son * linfáticos *, * maestro *, * Parameter Server * , * mensajes * , * Servicios *, * temas * , y * bolsas * , todos los cuales proporcionan datos a la gráfica de diferentes maneras.

Estos conceptos son implementados en el repositorio [ ros_comm ]( http://wiki.ros.org/ros_comm ) .

- [ ** Nodos ** ]( http://wiki.ros.org/Nodes ) : Los nodos son procesos que realizan el cálculo . ROS está diseñado para ser modular en una escala de grano fino ; un sistema de control del robot comprende por lo general muchos nodos . Por ejemplo , un nodo controla un telémetro de láser , un nodo controla los motores de las ruedas , un nodo lleva a cabo la localización , un nodo realiza planificación de trayectoria , un Nodo proporciona una vista gráfica del sistema , y así sucesivamente . Un nodo de ROS se escribe con el uso de un [ biblioteca de cliente ] ROS ( http://wiki.ros.org/Client % 20Libraries ) , como [ roscpp ]( http://wiki.ros.org/roscpp ) o [ Rospy ]( http://wiki.ros.org/rospy ) .
- [** Maestro **]( http://wiki.ros.org/Master ) : El ROS Master proporciona el registro de nombres y búsqueda para el resto de la Computación Gráfica . Sin el Maestro , los nodos no serían capaces de encontrar unos a otros mensajes de cambio, o invocar servicios .
- [** Parámetro Servidor **]( http://wiki.ros.org/Parameter % 20Server ) : El servidor de parámetros permite que los datos sean almacenados por la clave en una ubicación central . Actualmente forma parte del Máster .
- [Mensajes ** **]( http://wiki.ros.org/Messages ) : Los nodos se comunican entre sí mediante el paso [ mensajes ]( http://wiki.ros.org/Messages ) . Un mensaje es simplemente una estructura de datos , que comprende los campos con tipo. Se admiten los tipos primitivos estándar (entero , coma flotante , booleanos , etc ) , así como los arreglos de tipos primitivos. Los mensajes pueden incluir estructuras y arrays anidados arbitrariamente (parecido C estructuras) .
- [** Temas **]( http://wiki.ros.org/Topics ) : Los mensajes se enrutan a través de un sistema de transporte con la publicación / suscripción semántica. Un nodo envía un mensaje mediante su publicación en un determinado [ tema ]( http://wiki.ros.org/Topics ) . El tema es un [ nombre ]( http://wiki.ros.org/Names ) que se utiliza para identificar el contenido del mensaje . Un nodo que esté interesado en un determinado tipo de datos suscribirá el tema correspondiente. Puede haber múltiples editores y suscriptores concurrentes para un solo tema , y un único nodo puede publicar y / o suscribirse a múltiples temas. En general , los editores y suscriptores no son conscientes de la existencia de los demás. La idea es disociar la producción de información de su consumo . Lógicamente , se puede pensar en un tema como un bus de mensajes inflexible de tipos. Cada autobús tiene un nombre, y cualquier persona puede conectarse al bus para enviar o recibir mensajes mientras son del tipo correcto .
[]( http://ros.org/images/wiki/ROS_basic_concepts.png )

- [** Servicios **]( http://wiki.ros.org/Services ) : El modelo de publicación / suscripción es un paradigma de comunicación muy flexible , pero su - muchos-a- muchos, el transporte de ida no es apropiado para petición / respuesta interacciones , que a menudo se requieren en un sistema distribuido. Petición / respuesta se realiza a través de servicios , que se definen por un par de estructuras de mensajes : uno para la solicitud y uno por la respuesta. Un nodo que proporciona ofrece un servicio con un nombre y un cliente utiliza el servicio mediante el envío del mensaje de solicitud y en espera de la respuesta. Bibliotecas de cliente ROS generalmente presentan esta interacción para el programador como si fuera una llamada a procedimiento remoto.
- [** Bolsas **]( http://wiki.ros.org/Bags ) : Las bolsas son un formato para guardar y reproducir datos de mensajes de ROS . Las bolsas son un mecanismo importante para el almacenamiento de datos, como los datos de los sensores , que pueden ser difíciles de recoger , pero es necesario para desarrollar y probar algoritmos.

El ROS maestro actúa como un servicio de nombres en el ROS Computación Gráfica . Almacena temas y servicios de información de registro para los nodos ROS . Los nodos se comunican con el Maestro de reportar su información de registro. A medida que estos nodos se comunican con el Maestro , pueden recibir información sobre otros nodos inscritos y hacer las conexiones , según corresponda. El Maestro también hará devoluciones de llamada a estos nodos cuando se ejecutan estos cambios en la información de registro , lo que permite que los nodos para crear dinámicamente las conexiones como nuevos nodos.

Los nodos se conectan a otros nodos directamente ; el Maestro sólo proporciona información de búsqueda , al igual que un servidor DNS. Los nodos que se suscriben a un tema solicitarán las conexiones desde los nodos que publican ese tema , y se establecerá que la conexión a través de un acuerdo sobre la protocolo de conexión . El protocolo más común que se utiliza en un ROS se llama [ TCPROS ]( http://wiki.ros.org/ROS/TCPROS ), que utiliza sockets TCP / IP estándar .


Esta arquitectura permite una operación desconectada , donde los nombres son el medio principal por el que los sistemas más grandes y complejos se pueden construir . Los nombres tienen un papel muy importante en ROS: nodos , los temas , los servicios y todos los parámetros tienen nombre. Cada ROS soportes biblioteca cliente [ Reasignación de línea de comandos de nombres ]( http://wiki.ros.org/Remapping % 20Arguments ) , lo que significa un programa compilado se pueden reconfigurar en tiempo de ejecución para operar en una topología de Computación Gráfica diferente.

----

Por ejemplo, para controlar un telémetro láser Hokuyo , podemos iniciar el controlador hokuyo_node , que habla con el láser y publica sensor_msgs / messages LaserScan sobre el tema de análisis. Para procesar los datos , podríamos escribir un nodo utilizando laser_filters que se suscribe a los mensajes sobre el tema de análisis. Tras la suscripción , nuestro filtro comenzará automáticamente a recibir mensajes desde el láser.

Observe cómo se desacoplan los dos lados. Todo el nodo hokuyo_node hace es publicar exploraciones , sin el conocimiento de si alguien está suscrito . Todo el filtro no se suscriben a las exploraciones , sin el conocimiento de si alguien los está publicando . Los dos nodos se pueden iniciar, asesinados, y arrancará de nuevo en cualquier orden, sin inducir las condiciones de error .

Más tarde podríamos añadir otro láser a nuestro robot, así que tenemos que volver a configurar nuestro sistema. Todo lo que necesitamos hacer es volver a asignar los nombres que se utilizan . Cuando comenzamos nuestra primera hokuyo_node , podríamos decir que en lugar de reasignar exploración para base_scan , y hacer lo mismo con nuestro nodo de filtro. Ahora , tanto de estos nodos se comunican mediante el tema base_scan lugar y no escuchar los mensajes sobre el tema de exploración . Entonces sólo podemos comenzar otra hokuyo_node para el nuevo telémetro láser.



Nivel comunitario
-----------

Los conceptos ROS Comunidad de nivel son ROS recursos que permitan a las comunidades separadas para intercambiar software y conocimiento. Estos recursos incluyen :

- [**Distribuciones**]( http://wiki.ros.org/Distributions ) : ROS Distribuciones son colecciones de pilas versionados que se pueden instalar . Las distribuciones juegan un papel similar al de las distribuciones de Linux : hacen más fácil la instalación de una colección de software, y también mantienen versiones consistentes en un conjunto de software.
- [** Repositorios **]( http://wiki.ros.org/Repositories ) : ROS basa en una red federada de repositorios de código , en los que diferentes instituciones puedan desarrollar y lanzar sus propios componentes de software del robot.
- [** El ROS Wiki **]( http://wiki.ros.org/Documentation ) : La comunidad ROS Wiki es el principal foro para la documentación de información sobre ROS . Cualquier persona puede inscribirse para una cuenta y contribuir con su propia documentación , proporcionar correcciones o actualizaciones , escribir tutoriales y más.
- ** Sistema de Tickets Bug **: Por favor, vea [ Entradas ]( http://wiki.ros.org/Tickets ) para obtener información sobre las entradas de archivo.
- [** Listas de Correo **]( http://wiki.ros.org/Mailing % 20Lists ) : Los ros- lista de distribución es el principal canal de comunicación sobre los nuevos cambios a ROS , así como un foro para hacer preguntas sobre el software ROS .
- [** ROS Respuestas **]( http://answers.ros.org/ ) : Un Q & A sitio para responder a sus preguntas relacionadas con Ros- .
- [** Blog **]( http://www.willowgarage.com/blog ) : El [ Blog Willow Garage ]( http://www.willowgarage.com/blog ) proporciona actualizaciones regulares, incluyendo fotos y videos.

Nombrando
------

### Nombres Gráfico de recursos

Nombres de recursos Gráfica proporcionan una estructura de nombres jerárquico que se utiliza para todos los recursos en un ROS Computación Gráfica , como nodos , Parámetros , Temas, y Servicios. Estos nombres son muy poderosos en ROS y central a cómo los sistemas más grandes y complejas se componen de ROS, por lo que es fundamental para entender cómo estos nombres funcionan y cómo se pueden manipular .

Antes de describir los nombres más , he aquí algunos ejemplos de nombres :

- / ( El espacio de nombres global)
- / Foo
- / Stanford / robot / Nombre
- / Wg/node1

Nombres de recursos gráfica son un mecanismo importante en ROS para proporcionar encapsulación. Cada recurso se define dentro de un espacio de nombres , que se puede compartir con muchos otros recursos. En general , los recursos pueden crear recursos dentro de su espacio de nombres y que puedan acceder a los recursos dentro o por encima de su propio espacio de nombres. Se pueden hacer conexiones entre los recursos de los espacios de nombres distintos, pero eso se hace generalmente por código de integración por encima de los dos espacios de nombres. Esta encapsulación aísla diferentes partes del sistema de agarrar accidentalmente el recurso incorrecto nombrado o globalmente secuestrar nombres.

Los nombres se resuelven relativamente , por lo que los recursos no tienen que ser conscientes de que el espacio de nombres que están adentro Esto simplifica la programación como nodos que trabajan en conjunto se pueden escribir como si todos ellos están en el espacio de nombres de nivel superior. Cuando estos nodos están integrados en un sistema más grande , que pueden ser empujados hacia abajo en un espacio de nombres que define su colección de código . Por ejemplo , se podría tomar una demo de Stanford y una versión parcial de Willow Garage y fusionarlas en una nueva demo con stanford y subgrafos wg . Si las dos demos tuvieron un nodo llamado "cámara" , que no entren en conflicto . Herramientas de visualización gráfica (por ejemplo ), así como los parámetros ( por ejemplo demo_name ) que deben ser visibles para todo el gráfico se pueden crear nodos de nivel superior .

#### Nombres válidos

Un nombre válido tiene las siguientes características :

. 1 El primer carácter es un carácter alfabético ( [az | AZ] ) , tilde ( ~ ) o barra diagonal (/ )
. 2 caracteres siguientes pueden ser alfanumérica ( [ 0-9 | az | AZ] ) , guiones bajos ( _), o barras diagonales ( / )

------

** ** Excepción : los nombres de bases ( descritos a continuación) no puede tener barras diagonales (/) o tildes ( ~) en ellos.

------
#### Resolver

Hay cuatro tipos de recursos Nombres Gráfico en ROS : base , en relación , a nivel mundial , y privadas , que tienen la siguiente sintaxis :

- Base
- Relativa / Nombre
- / Global / Nombre
- ~ Privado / Nombre


Por defecto , la resolución se realiza en relación con el espacio de nombres del nodo. Por ejemplo , el nodo `/ wg/node1 ` tiene el espacio de nombres `/ wg ` , por lo que el nombre de nodo 2 se resolverá en `/ wg/node2 ` .

Nombres sin calificadores de espacio de nombres en absoluto son nombres de base . Nombres base son en realidad una subclase de nombres relativos y tienen las mismas reglas de resolución . Nombres de base se utilizan con mayor frecuencia para inicializar el nombre de nodo.

Los nombres que comienzan con un "/" son globales - que se consideran totalmente resueltos. * Los nombres globales deben evitarse tanto como sea posible , ya que limitan la portabilidad del código * .

Los nombres que comienzan con un "~" son privadas. Convierten el nombre del nodo en un espacio de nombres . Por ejemplo , el nodo 1 en espacio de nombres `/ wg /` tiene el espacio de nombres privado `/ wg/node1 ` . Nombres privados son útiles para pasar parámetros a un nodo específico a través del servidor de parámetros.

Estos son algunos ejemplos de resolución de nombres :

| Node | relativa ( por defecto) | Global | Privado |
| - | - | - | - |
| ` / Node1 ` | ` bar -> / bar ` | `/ bar -> / bar ` | `~ bar -> / node1/bar ` |
| ` / wg/node2 ` | ` bar -> / wg / bar ` | `/ bar -> / bar ` | `~ bar -> / wg/node2/bar ` |
| ` / Wg/node3 ` | ` foo / bar -> / wg / foo / bar ` | `/ foo / bar -> / foo / bar ` | `~ foo / bar -> / wg/node3/foo/bar `|

#### Reasignación

Cualquier nombre dentro de un nodo ROS se puede reasignar al iniciar el nodo en la línea de comandos. Para obtener más información sobre esta función, consulte [ Reasignación Argumentos ]( http://wiki.ros.org/Remapping % 20Arguments ) .

### Nombres de los recursos del paquete

Paquete Nombres de los recursos se utilizan en ROS con los conceptos del sistema de archivos de nivel para simplificar el proceso de hacer referencia a archivos y tipos de datos en el disco . Paquete Nombres de los recursos son muy simples : no son más que el nombre del paquete que el recurso está en más el nombre del recurso. Por ejemplo , el nombre ` std_msgs / Cadena ` se refiere a la "Cadena" tipo de mensaje en el " std_msgs " Package.

Algunos de los archivos relacionados con los ROS que se puede hacer referencia a la utilización del paquete Nombres de recursos incluyen:

- [ Mensaje ( msg) tipos ]( http://wiki.ros.org/msg )
- [ Servicio (SRV) tipos ]( http://wiki.ros.org/srv )
- [ Tipos de nodo ]( http://wiki.ros.org/Nodes )

Paquete Nombres de los recursos son muy similares a presentar sendas , excepto que son mucho más cortos . Esto es debido a la capacidad de ROS para localizar paquetes en el disco y hacer suposiciones adicionales acerca de su contenido . Por ejemplo , las descripciones de mensajes siempre se almacenan en el subdirectorio ` msg ` y tienen la extensión ` . ` Msg, por lo que ` std_msgs / Cadena ` es la abreviatura de ` ruta / al / std_msgs / msg / String.msg ` . Del mismo modo , el tipo de nodo ` foo / bar ` es equivalente a la búsqueda de un archivo llamado ` bar ` en el paquete ` foo ` con permisos de ejecución .

#### Nombres válidos

Nombres de los recursos del paquete tienen estrictas reglas de nomenclatura , ya que a menudo se utilizan en el código generado automáticamente. Por esta razón, un paquete de ROS no puede tener caracteres especiales que no sean un guión bajo , y deben comenzar con un carácter alfabético . Un nombre válido tiene las siguientes características :

1. El primer carácter es un carácter alfabético ( [az | AZ] )
2. caracteres siguientes pueden ser alfanumérica ( [ 0-9 | az | AZ] ) , guiones bajos ( _ ) o una barra diagonal (/ )
3. Hay a lo sumo una barra ( '/') .

