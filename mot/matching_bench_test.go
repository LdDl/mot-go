package mot

import (
	"fmt"
	"testing"
)

type matchingBenchmarkCase struct {
	name    string
	count   int
	spacing float64
}

func BenchmarkTrackers(b *testing.B) {
	cases := []matchingBenchmarkCase{
		{"Separated100", 100, 1000},
		{"Dense100", 100, 0.05},
		{"Dense300", 300, 0.05},
	}
	for _, scene := range cases {
		for _, kind := range []string{"Simple", "IoU", "ByteGreedy", "ByteHungarian"} {
			b.Run(fmt.Sprintf("%s/%s", kind, scene.name), func(b *testing.B) {
				b.ReportAllocs()
				for i := 0; i < b.N; i++ {
					b.StopTimer()
					tracks := make([]*SimpleBlob, scene.count)
					detections := make([]*SimpleBlob, scene.count)
					confidences := make([]float64, scene.count)
					for j := range tracks {
						x := 100 + float64(j)*scene.spacing
						tracks[j] = NewSimpleBlobWithTime(NewRect(x, 100, 100, 100), 0.04)
						detections[j] = NewSimpleBlobWithTime(NewRect(x+1, 100, 100, 100), 0.04)
						confidences[j] = 0.9
					}
					var match func([]*SimpleBlob) error
					switch kind {
					case "Simple":
						tracker := NewSimpleTrackerDefault[*SimpleBlob]()
						for _, blob := range tracks {
							tracker.Objects[blob.GetID()] = blob
						}
						match = tracker.MatchObjects
					case "IoU":
						tracker := NewIoUTracker[*SimpleBlob](75, 0.3)
						for _, blob := range tracks {
							tracker.Objects[blob.GetID()] = blob
						}
						match = tracker.MatchObjects
					default:
						algorithm := MatchingAlgorithmGreedy
						if kind == "ByteHungarian" {
							algorithm = MatchingAlgorithmHungarian
						}
						tracker := NewByteTracker[*SimpleBlob](75, 0.3, 0.5, 0.3, algorithm)
						for _, blob := range tracks {
							tracker.Objects[blob.GetID()] = blob
						}
						match = func(frame []*SimpleBlob) error { return tracker.MatchObjects(frame, confidences) }
					}
					b.StartTimer()
					if err := match(detections); err != nil {
						b.Fatal(err)
					}
				}
			})
		}
	}
}
