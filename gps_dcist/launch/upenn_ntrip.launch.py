from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable

# <arg name="host"                   default = "ppcp.grasp.upenn.edu" />
# <arg name="port"                   default = "2101" />
# <arg name="mountpoint"             default = "U-BLOX" />
# <arg name="authenticate"           default = "true" />
# <arg name="username"               default = "dcist" />
# <arg name="password"               default = "Fernando" />
# <arg name="ntrip_version"          default = "" />
# <arg name="ssl"                    default = "false" />
# <arg name="cert"                   default = "" />
# <arg name="key"                    default = "" />
# <arg name="ca_cert"                default = "" />
# <arg name="debug"                  default = "false" />
# <arg name="rtcm_message_package"   default = "mavros_msgs" />

def generate_launch_description():
      return LaunchDescription([
          # Declare arguments with default values
          DeclareLaunchArgument('namespace',             default_value=''),
          DeclareLaunchArgument('node_name',             default_value='ntrip_client'),
          DeclareLaunchArgument('host',                  default_value='192.168.129.25'), #ppcp.grasp.upenn.edu
          DeclareLaunchArgument('port',                  default_value='2101'),
          DeclareLaunchArgument('mountpoint',            default_value='U-BLOX'),
          DeclareLaunchArgument('authenticate',          default_value='True'),
          DeclareLaunchArgument('username',              default_value='dcist'),
          DeclareLaunchArgument('password',              default_value='Fernando'),
          DeclareLaunchArgument('ntrip_version',         default_value='None'),
          DeclareLaunchArgument('ssl',                   default_value='False'),
          DeclareLaunchArgument('cert',                  default_value='None'),
          DeclareLaunchArgument('key',                   default_value='None'),
          DeclareLaunchArgument('ca_cert',               default_value='None'),
          DeclareLaunchArgument('debug',                 default_value='True'),
          DeclareLaunchArgument('rtcm_message_package',  default_value='rtcm_msgs'),
          #DeclareLaunchArgument('rtcm_message_package',  default_value='mavros_msgs'), 

          # Pass an environment variable to the node
          SetEnvironmentVariable(name='NTRIP_CLIENT_DEBUG', value=LaunchConfiguration('debug')),

          # ******************************************************************
          # NTRIP Client Node
          # ******************************************************************
          Node(
                name=LaunchConfiguration('node_name'),
                namespace=LaunchConfiguration('namespace'),
                #namespace='',
                package='ntrip_client',
                executable='ntrip_ros.py',
                parameters=[
                  {
                    # Required parameters used to connect to the NTRIP server
                    'host': LaunchConfiguration('host'),
                    'port': LaunchConfiguration('port'),
                    'mountpoint': LaunchConfiguration('mountpoint'),

                    # Optional parameter that will set the NTRIP version in the initial HTTP request to the NTRIP caster.
                    'ntrip_version': LaunchConfiguration('ntrip_version'),

                    # If this is set to true, we will read the username and password and attempt to authenticate. If not, we will attempt to connect unauthenticated
                    'authenticate': LaunchConfiguration('authenticate'),

                    # If authenticate is set the true, we will use these to authenticate with the server
                    'username': LaunchConfiguration('username'),
                    'password': LaunchConfiguration('password'),

                    # Whether to connect with SSL. cert, key, and ca_cert options will only take effect if this is true
                    'ssl': LaunchConfiguration('ssl'),

                    # If the NTRIP caster uses cert based authentication, you can specify the cert and keys to use with these options
                    'cert': LaunchConfiguration('cert'),
                    'key':  LaunchConfiguration('key'),

                    # If the NTRIP caster uses self signed certs, or you need to use a different CA chain, specify the path to the file here
                    'ca_cert': LaunchConfiguration('ca_cert'),

                    # Not sure if this will be looked at by other ndoes, but this frame ID will be added to the RTCM messages published by this node
                    'rtcm_frame_id': 'odom',

                    # Optional parameters that will allow for longer or shorter NMEA messages. Standard max length for NMEA is 82
                    'nmea_max_length': 128,
                    'nmea_min_length': 3,

                    # Use this parameter to change the type of RTCM message published by the node. Defaults to "mavros_msgs", but we also support "rtcm_msgs"
                    'rtcm_message_package': LaunchConfiguration('rtcm_message_package'),

                    # Will affect how many times the node will attempt to reconnect before exiting, and how long it will wait in between attempts when a reconnect occurs
                    'reconnect_attempt_max': 10,
                    'reconnect_attempt_wait_seconds': 5,

                    # How many seconds is acceptable in between receiving RTCM. If RTCM is not received for this duration, the node will attempt to reconnect
                    'rtcm_timeout_seconds': 4
                  }
                ],
                # Uncomment the following section and replace "/gx5/nmea/sentence" with the topic you are sending NMEA on if it is not the one we requested
                #remappings=[
                #  ("nmea", "/gx5/nmea/sentence")
                #],
          )
      ])
