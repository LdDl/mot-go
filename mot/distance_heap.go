package mot

import "github.com/google/uuid"

type distanceBlob[B Blob[B]] struct {
	underlying B
	id         uuid.UUID
	distance   float64
}

// Copied from container/heap - https://golang.org/pkg/container/heap/
// Why make copy? Just want to avoid type conversion

type distanceHeap[B Blob[B]] []*distanceBlob[B]

func (h distanceHeap[B]) Len() int           { return len(h) }
func (h distanceHeap[B]) Less(i, j int) bool { return h[i].distance < h[j].distance }
func (h distanceHeap[B]) Swap(i, j int)      { h[i], h[j] = h[j], h[i] }

// Push pushes the element x onto the heap.
// The complexity is O(log n) where n = h.Len().
func (h *distanceHeap[B]) Push(x *distanceBlob[B]) {
	*h = append(*h, x)
	h.up(h.Len() - 1)
}

// Pop removes and returns the minimum element (according to Less) from the heap.
// The complexity is O(log n) where n = h.Len().
// Pop is equivalent to Remove(h, 0).
func (h *distanceHeap[B]) Pop() *distanceBlob[B] {
	n := h.Len() - 1
	h.Swap(0, n)
	h.down(0, n)
	heapSize := len(*h)
	lastNode := (*h)[heapSize-1]
	*h = (*h)[0 : heapSize-1]
	return lastNode
}

// Remove removes and returns the element at index i from the heap.
// The complexity is O(log n) where n = h.Len().
func (h distanceHeap[B]) Remove(i int) *distanceBlob[B] {
	n := h.Len() - 1
	if n != i {
		h.Swap(i, n)
		if !h.down(i, n) {
			h.up(i)
		}
	}
	return h.Pop()
}

func (h distanceHeap[B]) up(j int) {
	for {
		i := (j - 1) / 2
		if i == j || !h.Less(j, i) {
			break
		}
		h.Swap(i, j)
		j = i
	}
}

func (h distanceHeap[B]) down(i0, n int) bool {
	i := i0
	for {
		j1 := 2*i + 1
		if j1 >= n || j1 < 0 {
			break
		}
		j := j1
		if j2 := j1 + 1; j2 < n && h.Less(j2, j1) {
			j = j2
		}
		if !h.Less(j, i) {
			break
		}
		h.Swap(i, j)
		i = j
	}
	return i > i0
}
