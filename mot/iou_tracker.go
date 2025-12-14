package mot

import (
	"container/heap"

	"github.com/google/uuid"
)

// IoUTracker is a naive implementation of Multi-object tracker (MOT) with IoU matching.
// Uses hybrid IoU + distance matching for better recovery when IoU is zero.
type IoUTracker[B Blob[B]] struct {
	// Max no match (max number of frames when object could not be found again)
	maxNoMatch int
	// IoU threshold for matching
	iouThreshold float64
	// Storage for tracked objects
	Objects map[uuid.UUID]B
}

// NewDefaultIoUTracker creates a default instance of IoUTracker.
// Default values: maxNoMatch=75, iouThreshold=0.0
func NewDefaultIoUTracker[B Blob[B]]() *IoUTracker[B] {
	return &IoUTracker[B]{
		maxNoMatch:   75,
		iouThreshold: 0.0,
		Objects:      make(map[uuid.UUID]B),
	}
}

// NewIoUTracker creates a new instance of IoUTracker with specified parameters.
func NewIoUTracker[B Blob[B]](maxNoMatch int, iouThreshold float64) *IoUTracker[B] {
	return &IoUTracker[B]{
		maxNoMatch:   maxNoMatch,
		iouThreshold: iouThreshold,
		Objects:      make(map[uuid.UUID]B),
	}
}

// iouDistanceBlob holds a blob with its match score and target ID for priority queue
type iouDistanceBlob[B Blob[B]] struct {
	score  float64
	minID  uuid.UUID
	blob   B
	index  int
}

// iouHeap implements heap.Interface for max-heap by score
type iouHeap[B Blob[B]] []*iouDistanceBlob[B]

func (h iouHeap[B]) Len() int { return len(h) }

// Less returns true if i has higher score (max-heap)
func (h iouHeap[B]) Less(i, j int) bool { return h[i].score > h[j].score }

func (h iouHeap[B]) Swap(i, j int) {
	h[i], h[j] = h[j], h[i]
	h[i].index = i
	h[j].index = j
}

func (h *iouHeap[B]) Push(x any) {
	n := len(*h)
	item := x.(*iouDistanceBlob[B])
	item.index = n
	*h = append(*h, item)
}

func (h *iouHeap[B]) Pop() any {
	old := *h
	n := len(old)
	item := old[n-1]
	old[n-1] = nil
	item.index = -1
	*h = old[0 : n-1]
	return item
}

// MatchObjects matches new detections to existing tracked objects using hybrid IoU + distance.
func (tracker *IoUTracker[B]) MatchObjects(newObjects []B) error {
	// Mark all existing objects as deactivated
	for _, object := range tracker.Objects {
		object.Deactivate()
	}

	blobsToRegister := make(map[uuid.UUID]B)

	// Build priority queue with IoU/distance scores
	pq := &iouHeap[B]{}
	heap.Init(pq)

	for i := range newObjects {
		newObj := newObjects[i]
		var maxID uuid.UUID
		maxScore := 0.0

		// Hybrid IoU + Distance matching
		for objID, object := range tracker.Objects {
			predictedBBox := object.GetPredictedBBox()
			iouValue := IoU(newObj.GetBBox(), predictedBBox)

			// Add distance-based fallback
			predictedCenter := Point{
				X: predictedBBox.X + predictedBBox.Width/2.0,
				Y: predictedBBox.Y + predictedBBox.Height/2.0,
			}
			distance := euclideanDistance(predictedCenter, newObj.GetCenter())

			// Convert to 0-1 similarity
			distanceScore := 1.0 / (1.0 + distance*0.01)

			// Combine IoU and distance (favor IoU when available, fallback to distance)
			var combinedScore float64
			if iouValue > 0.05 {
				combinedScore = iouValue*0.8 + distanceScore*0.2
			} else {
				// Lower weight for pure distance matching
				combinedScore = distanceScore * 0.5
			}

			if combinedScore > maxScore {
				maxScore = combinedScore
				maxID = objID
			}
		}

		item := &iouDistanceBlob[B]{
			score: maxScore,
			minID: maxID,
			blob:  newObj,
		}
		heap.Push(pq, item)
	}

	// Prevent double update of objects
	reservedObjects := make(map[uuid.UUID]bool)

	// Process matches from highest score to lowest
	for pq.Len() > 0 {
		item := heap.Pop(pq).(*iouDistanceBlob[B])
		maxScore := item.score
		minID := item.minID
		blob := item.blob

		// Check if object is already reserved
		if reservedObjects[minID] {
			// Register as new object
			blobsToRegister[blob.GetID()] = blob
			continue
		}

		// Filter by IoU threshold
		if maxScore > tracker.iouThreshold {
			if existingObj, ok := tracker.Objects[minID]; ok {
				// Advance time and update in correct order
				existingObj.PredictNextPosition()
				err := existingObj.Update(blob)
				if err != nil {
					return err
				}
				existingObj.ResetNoMatch()

				// Update ID of new object to match existing one
				blob.SetID(minID)
				reservedObjects[minID] = true
			}
		} else {
			// Register object as a new one
			blobsToRegister[blob.GetID()] = blob
		}
	}

	// Add new objects to tracker
	for id, blob := range blobsToRegister {
		tracker.Objects[id] = blob
	}

	// Handle unmatched objects (predict forward for track maintenance)
	for id, object := range tracker.Objects {
		if !reservedObjects[id] {
			object.PredictNextPosition()
			object.IncNoMatch()
		}
	}

	// Clean up existing data - remove objects not found for a long time
	for id, object := range tracker.Objects {
		if object.GetNoMatchTimes() > tracker.maxNoMatch {
			delete(tracker.Objects, id)
		}
	}

	return nil
}
