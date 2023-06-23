package mot

import (
	"image"
	"math"
)

type Rectangle struct {
	X      float64
	Y      float64
	Width  float64
	Height float64
}

func NewRect(x, y, height, width float64) Rectangle {
	return Rectangle{
		X:      x,
		Y:      y,
		Width:  height,
		Height: width,
	}
}

func NewRectFrom(rect image.Rectangle) Rectangle {
	return Rectangle{
		X:      float64(rect.Min.X),
		Y:      float64(rect.Min.Y),
		Width:  float64(rect.Dx()),
		Height: float64(rect.Dy()),
	}
}

type Point struct {
	X float64
	Y float64
}

func NewPoint(x, y float64) Point {
	return Point{
		X: x,
		Y: y,
	}
}

func NewPointFrom(point image.Point) Point {
	return Point{
		X: float64(point.X),
		Y: float64(point.Y),
	}
}

func euclideanDistance(p1, p2 Point) float64 {
	return math.Sqrt(math.Pow(float64(p1.X-p2.X), 2) + math.Pow(float64(p1.Y-p2.Y), 2))
}
