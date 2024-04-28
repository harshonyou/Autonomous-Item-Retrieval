# A list of processed items, each containing both original characteristics and additional processing results.

# Standard message header containing timestamp and frame information.
std_msgs/Header header # timestamp and frame this data is associated with

# An array of ProcessedItem messages, each representing an item and its processing results.
solution_interfaces/ProcessedItem[] data