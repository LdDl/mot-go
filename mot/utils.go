package mot

// IoU calculates Intersection over Union between two rectangles.
// This should be consistent with your geom_f64.Rectangle and utils.iou.
// Assuming Rectangle has X, Y, Width, Height fields.
func IoU(r1, r2 Rectangle) float64 {
	xA := maxFloat64(r1.X, r2.X)
	yA := maxFloat64(r1.Y, r2.Y)
	xB := minFloat64(r1.X+r1.Width, r2.X+r2.Width)
	yB := minFloat64(r1.Y+r1.Height, r2.Y+r2.Height)

	interArea := maxFloat64(0, xB-xA) * maxFloat64(0, yB-yA)
	if interArea == 0 {
		return 0.0
	}

	r1Area := r1.Width * r1.Height
	r2Area := r2.Width * r2.Height

	iouVal := interArea / (r1Area + r2Area - interArea)
	// isnan and isinf checks might be needed if areas can be zero or negative,
	// but with positive width/height and interArea check, it should be fine.
	return iouVal
}

func maxFloat64(a, b float64) float64 {
	if a > b {
		return a
	}
	return b
}

func minFloat64(a, b float64) float64 {
	if a < b {
		return a
	}
	return b
}

func maxInt(a, b int) int {
	if a > b {
		return a
	}
	return b
}
