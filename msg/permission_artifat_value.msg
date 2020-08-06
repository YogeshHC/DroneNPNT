#This message is used to share the information regarding the parsed permission artifat to the system.

uint64 timestamp

uint64 start_time		# Start time of flight specified in the permission artifat

uint64 end_time			# End time of flight specified in the permission artifat

float32[20] geo_fence		# Geofence in  {lat1, long1 , lat2,long2 ,,,,} format from permission artifat

uint8 fence_size		# Geofence size to iterate how many pairs of lat longs present in geo_fence

uint32[120] pa_name






