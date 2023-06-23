package mot

import (
	"math"
	"testing"
)

const (
	eps = 0.00001
)

func TestEuclideanDistance(t *testing.T) {
	p1 := Point{X: 341, Y: 264}
	p2 := Point{X: 421, Y: 427}
	correnctAnswer := 181.57367
	answer := euclideanDistance(p1, p2)
	if math.Abs(answer-correnctAnswer) > eps {
		t.Errorf("Wrong answer: %v, correct answer: %v", answer, correnctAnswer)
	}
}
