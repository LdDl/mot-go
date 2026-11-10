package mot

func trimTrack(track []Point, limit int) []Point {
	if len(track) > limit {
		copy(track, track[len(track)-limit:])
		track = track[:limit]
	}
	return track
}

func appendTrackPoint(track []Point, limit int, point Point) []Point {
	if limit == 0 {
		return track
	}
	if len(track) == limit {
		copy(track, track[1:])
		track = track[:limit-1]
	}
	return append(track, point)
}
