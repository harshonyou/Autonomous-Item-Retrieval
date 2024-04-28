# A list of peers, typically used to share information about multiple agents in the environment.

# Standard message header containing timestamp and frame information.
std_msgs/Header header # timestamp and frame this data is associated with

# An array of Peers messages, each representing the position of a peer.
solution_interfaces/Peers[] data # list of peer positions