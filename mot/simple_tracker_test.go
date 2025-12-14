package mot

import (
	"encoding/csv"
	"fmt"
	"os"
	"strings"
	"testing"
)

func TestMatchObjectsSpread(t *testing.T) {
	bboxesIterations := [][]Rectangle{
		// Each nested vector represents set of bounding boxes on a single frame
		{NewRect(378.0, 147.0, 173.0, 243.0)},
		{NewRect(374.0, 147.0, 180.0, 253.0)},
		{NewRect(375.0, 154.0, 178.0, 256.0)},
		{NewRect(376.0, 162.0, 177.0, 267.0)},
		{NewRect(375.0, 166.0, 178.0, 268.0)},
		{NewRect(375.0, 177.0, 186.0, 266.0)},
		{NewRect(370.0, 185.0, 197.0, 273.0)},
		{NewRect(363.0, 209.0, 203.0, 264.0)},
		{NewRect(70.0, 14.0, 227.0, 254.0), NewRect(364.0, 214.0, 200.0, 262.0)},
		{NewRect(365.0, 218.0, 205.0, 263.0)},
		{NewRect(67.0, 23.0, 236.0, 246.0), NewRect(366.0, 231.0, 209.0, 260.0)},
		{NewRect(73.0, 18.0, 227.0, 264.0), NewRect(610.0, 47.0, 324.0, 355.0), NewRect(370.0, 238.0, 199.0, 259.0), NewRect(381.0, -1.0, 103.0, 60.0)},
		{NewRect(67.0, 16.0, 229.0, 271.0), NewRect(370.0, 250.0, 195.0, 264.0), NewRect(381.0, -2.0, 106.0, 58.0)},
		{NewRect(62.0, 15.0, 233.0, 268.0), NewRect(365.0, 257.0, 205.0, 264.0), NewRect(379.0, -1.0, 109.0, 59.0)},
		{NewRect(60.0, 7.0, 234.0, 279.0), NewRect(360.0, 269.0, 212.0, 260.0), NewRect(380.0, -1.0, 109.0, 60.0)},
		{NewRect(50.0, 41.0, 251.0, 295.0), NewRect(619.0, 25.0, 308.0, 399.0), NewRect(361.0, 276.0, 215.0, 265.0), NewRect(380.0, -1.0, 110.0, 63.0)},
		{NewRect(48.0, 36.0, 242.0, 302.0), NewRect(622.0, 21.0, 299.0, 411.0), NewRect(357.0, 283.0, 222.0, 255.0), NewRect(379.0, 0.0, 113.0, 64.0)},
		{NewRect(41.0, 28.0, 245.0, 319.0), NewRect(625.0, 31.0, 308.0, 392.0), NewRect(350.0, 306.0, 239.0, 231.0), NewRect(377.0, 0.0, 116.0, 65.0)},
		{NewRect(630.0, 98.0, 294.0, 324.0), NewRect(346.0, 310.0, 250.0, 239.0), NewRect(378.0, 0.0, 112.0, 65.0)},
		{NewRect(636.0, 99.0, 290.0, 323.0), NewRect(344.0, 320.0, 254.0, 229.0), NewRect(378.0, 2.0, 114.0, 65.0)},
		{NewRect(636.0, 103.0, 295.0, 318.0), NewRect(347.0, 332.0, 251.0, 211.0)},
		{NewRect(362.0, 1.0, 147.0, 90.0), NewRect(637.0, 104.0, 292.0, 321.0), NewRect(337.0, 344.0, 272.0, 196.0)},
		{NewRect(360.0, -2.0, 152.0, 97.0), NewRect(12.0, 74.0, 237.0, 324.0), NewRect(639.0, 104.0, 293.0, 316.0), NewRect(347.0, 350.0, 258.0, 185.0)},
		{NewRect(361.0, -4.0, 149.0, 99.0), NewRect(9.0, 112.0, 251.0, 313.0), NewRect(627.0, 106.0, 314.0, 321.0)},
		{NewRect(360.0, -3.0, 151.0, 99.0), NewRect(15.0, 115.0, 231.0, 311.0), NewRect(633.0, 91.0, 297.0, 346.0)},
		{NewRect(362.0, -7.0, 148.0, 106.0), NewRect(10.0, 109.0, 241.0, 320.0), NewRect(639.0, 93.0, 294.0, 347.0)},
		{NewRect(362.0, -9.0, 146.0, 109.0), NewRect(12.0, 109.0, 233.0, 326.0), NewRect(639.0, 95.0, 288.0, 347.0)},
		// {NewRect(362.0,-9.0,147.0,111.0), NewRect(3.0,103.0,236.0,346.0), NewRect(645.0,98.0,281.0,343.0)}, // here one of blobs disappears
		// {NewRect(365.0,-10.0,143.0,114.0), NewRect(645.0,99.0,283.0,345.0), NewRect(9.0,141.0,238.0,323.0)},
	}

	tracker := NewNewSimpleTracker[*SimpleBlob](15.0, 5)
	dt := 1.0 / 25.0 // emulate 25 fps

	for _, iteration := range bboxesIterations {
		blobs := make([]*SimpleBlob, len(iteration))
		for j, bbox := range iteration {
			blob := NewSimpleBlobWithTime(bbox, dt)
			blobs[j] = blob
		}
		err := tracker.MatchObjects(blobs)
		if err != nil {
			t.Error(err)
			return
		}
	}

	correctNumOfObjects := 4
	numOfObjects := len(tracker.Objects)
	if numOfObjects != correctNumOfObjects {
		t.Errorf("incorrect number of objects: %d, expected: %d", numOfObjects, correctNumOfObjects)
		return
	}

	// Optional: CSV output
	file, err := os.Create("../data/blobs_spread.csv")
	if err != nil {
		t.Error(err)
		return
	}
	defer file.Close()

	writer := csv.NewWriter(file)
	defer writer.Flush()
	writer.Comma = ';'

	err = writer.Write([]string{"id", "track"})
	if err != nil {
		t.Error(err)
		return
	}

	for objectID, object := range tracker.Objects {
		track := object.GetTrack()
		data := make([]string, len(track))
		for idx, pt := range track {
			data[idx] = fmt.Sprintf("%f,%f", pt.X, pt.Y)
		}
		dataStr := strings.Join(data, "|")
		err = writer.Write([]string{objectID.String(), dataStr})
		if err != nil {
			t.Error(err)
			return
		}
	}
}

func TestMatchObjectsSimilar(t *testing.T) {

	bboxesOne := [][]float64{[]float64{236, -25, 386, 35}, []float64{237, -24, 387, 36}, []float64{238, -22, 388, 38}, []float64{236, -20, 386, 40}, []float64{236, -19, 386, 41}, []float64{237, -18, 387, 42}, []float64{237, -18, 387, 42}, []float64{238, -17, 388, 43}, []float64{237, -14, 387, 46}, []float64{237, -14, 387, 46}, []float64{237, -12, 387, 48}, []float64{237, -12, 387, 48}, []float64{237, -11, 387, 49}, []float64{237, -11, 387, 49}, []float64{237, -10, 387, 50}, []float64{237, -10, 387, 50}, []float64{237, -8, 387, 52}, []float64{237, -8, 387, 52}, []float64{236, -7, 386, 53}, []float64{236, -7, 386, 53}, []float64{236, -6, 386, 54}, []float64{236, -6, 386, 54}, []float64{236, -2, 386, 58}, []float64{235, 0, 385, 60}, []float64{236, 2, 386, 62}, []float64{236, 5, 386, 65}, []float64{236, 9, 386, 69}, []float64{235, 12, 385, 72}, []float64{235, 14, 385, 74}, []float64{233, 16, 383, 76}, []float64{232, 26, 382, 86}, []float64{233, 28, 383, 88}, []float64{233, 40, 383, 100}, []float64{233, 30, 383, 90}, []float64{232, 22, 382, 82}, []float64{232, 34, 382, 94}, []float64{232, 21, 382, 81}, []float64{233, 40, 383, 100}, []float64{232, 40, 382, 100}, []float64{232, 40, 382, 100}, []float64{232, 36, 382, 96}, []float64{232, 53, 382, 113}, []float64{232, 50, 382, 110}, []float64{233, 55, 383, 115}, []float64{232, 50, 382, 110}, []float64{234, 68, 384, 128}, []float64{231, 49, 381, 109}, []float64{232, 68, 382, 128}, []float64{231, 31, 381, 91}, []float64{232, 64, 382, 124}, []float64{233, 71, 383, 131}, []float64{231, 64, 381, 124}, []float64{231, 74, 381, 134}, []float64{231, 64, 381, 124}, []float64{230, 77, 380, 137}, []float64{232, 82, 382, 142}, []float64{232, 78, 382, 138}, []float64{232, 78, 382, 138}, []float64{231, 79, 381, 139}, []float64{231, 79, 381, 139}, []float64{231, 91, 381, 151}, []float64{232, 78, 382, 138}, []float64{232, 78, 382, 138}, []float64{233, 90, 383, 150}, []float64{232, 92, 382, 152}, []float64{232, 92, 382, 152}, []float64{233, 98, 383, 158}, []float64{232, 100, 382, 160}, []float64{231, 92, 381, 152}, []float64{233, 110, 383, 170}, []float64{234, 92, 384, 152}, []float64{234, 92, 384, 152}, []float64{234, 110, 384, 170}, []float64{234, 92, 384, 152}, []float64{233, 104, 383, 164}, []float64{234, 111, 384, 171}, []float64{234, 106, 384, 166}, []float64{234, 106, 384, 166}, []float64{233, 124, 383, 184}, []float64{236, 125, 386, 185}, []float64{236, 125, 386, 185}, []float64{232, 120, 382, 180}, []float64{236, 131, 386, 191}, []float64{232, 132, 382, 192}, []float64{238, 139, 388, 199}, []float64{236, 141, 386, 201}, []float64{232, 151, 382, 211}, []float64{236, 145, 386, 205}, []float64{236, 145, 386, 205}, []float64{231, 133, 381, 193}, []float64{237, 148, 387, 208}, []float64{237, 148, 387, 208}, []float64{237, 148, 387, 208}, []float64{237, 148, 387, 208}, []float64{237, 148, 387, 208}, []float64{237, 148, 387, 208}, []float64{237, 148, 387, 208}, []float64{237, 148, 387, 208}, []float64{237, 148, 387, 208}, []float64{237, 148, 387, 208}, []float64{237, 148, 387, 208}, []float64{237, 148, 387, 208}, []float64{237, 148, 387, 208}, []float64{237, 148, 387, 208}, []float64{237, 148, 387, 208}, []float64{237, 148, 387, 208}, []float64{237, 148, 387, 208}, []float64{237, 148, 387, 208}, []float64{237, 148, 387, 208}, []float64{237, 148, 387, 208}, []float64{237, 148, 387, 208}, []float64{237, 148, 387, 208}}
	bboxesTwo := [][]float64{[]float64{321, -25, 471, 35}, []float64{322, -24, 472, 36}, []float64{323, -22, 473, 38}, []float64{321, -20, 471, 40}, []float64{321, -19, 471, 41}, []float64{322, -18, 472, 42}, []float64{322, -18, 472, 42}, []float64{323, -17, 473, 43}, []float64{322, -14, 472, 46}, []float64{322, -14, 472, 46}, []float64{322, -12, 472, 48}, []float64{322, -12, 472, 48}, []float64{322, -11, 472, 49}, []float64{322, -11, 472, 49}, []float64{322, -10, 472, 50}, []float64{322, -10, 472, 50}, []float64{322, -8, 472, 52}, []float64{322, -8, 472, 52}, []float64{321, -7, 471, 53}, []float64{321, -7, 471, 53}, []float64{321, -6, 471, 54}, []float64{321, -6, 471, 54}, []float64{321, -2, 471, 58}, []float64{320, 0, 470, 60}, []float64{321, 2, 471, 62}, []float64{321, 5, 471, 65}, []float64{321, 9, 471, 69}, []float64{320, 12, 470, 72}, []float64{320, 14, 470, 74}, []float64{318, 16, 468, 76}, []float64{317, 26, 467, 86}, []float64{318, 28, 468, 88}, []float64{318, 40, 468, 100}, []float64{318, 30, 468, 90}, []float64{317, 22, 467, 82}, []float64{317, 34, 467, 94}, []float64{317, 21, 467, 81}, []float64{318, 40, 468, 100}, []float64{317, 40, 467, 100}, []float64{317, 40, 467, 100}, []float64{317, 36, 467, 96}, []float64{317, 53, 467, 113}, []float64{317, 50, 467, 110}, []float64{318, 55, 468, 115}, []float64{317, 50, 467, 110}, []float64{319, 68, 469, 128}, []float64{316, 49, 466, 109}, []float64{317, 68, 467, 128}, []float64{316, 31, 466, 91}, []float64{317, 64, 467, 124}, []float64{318, 71, 468, 131}, []float64{316, 64, 466, 124}, []float64{316, 74, 466, 134}, []float64{316, 64, 466, 124}, []float64{315, 77, 465, 137}, []float64{317, 82, 467, 142}, []float64{317, 78, 467, 138}, []float64{317, 78, 467, 138}, []float64{316, 79, 466, 139}, []float64{316, 79, 466, 139}, []float64{316, 91, 466, 151}, []float64{317, 78, 467, 138}, []float64{317, 78, 467, 138}, []float64{318, 90, 468, 150}, []float64{317, 92, 467, 152}, []float64{317, 92, 467, 152}, []float64{318, 98, 468, 158}, []float64{317, 100, 467, 160}, []float64{316, 92, 466, 152}, []float64{318, 110, 468, 170}, []float64{319, 92, 469, 152}, []float64{319, 92, 469, 152}, []float64{319, 110, 469, 170}, []float64{319, 92, 469, 152}, []float64{318, 104, 468, 164}, []float64{319, 111, 469, 171}, []float64{319, 106, 469, 166}, []float64{319, 106, 469, 166}, []float64{318, 124, 468, 184}, []float64{321, 125, 471, 185}, []float64{321, 125, 471, 185}, []float64{317, 120, 467, 180}, []float64{321, 131, 471, 191}, []float64{317, 132, 467, 192}, []float64{323, 139, 473, 199}, []float64{321, 141, 471, 201}, []float64{317, 151, 467, 211}, []float64{321, 145, 471, 205}, []float64{321, 145, 471, 205}, []float64{316, 133, 466, 193}, []float64{322, 148, 472, 208}, []float64{322, 148, 472, 208}, []float64{322, 148, 472, 208}, []float64{322, 148, 472, 208}, []float64{322, 148, 472, 208}, []float64{322, 148, 472, 208}, []float64{322, 148, 472, 208}, []float64{322, 148, 472, 208}, []float64{322, 148, 472, 208}, []float64{322, 148, 472, 208}, []float64{322, 148, 472, 208}, []float64{322, 148, 472, 208}, []float64{322, 148, 472, 208}, []float64{322, 148, 472, 208}, []float64{322, 148, 472, 208}, []float64{322, 148, 472, 208}, []float64{322, 148, 472, 208}, []float64{322, 148, 472, 208}, []float64{322, 148, 472, 208}, []float64{322, 148, 472, 208}, []float64{322, 148, 472, 208}, []float64{322, 148, 472, 208}}
	bboxesThree := [][]float64{[]float64{151, -25, 301, 35}, []float64{152, -24, 302, 36}, []float64{153, -22, 303, 38}, []float64{151, -20, 301, 40}, []float64{151, -19, 301, 41}, []float64{152, -18, 302, 42}, []float64{152, -18, 302, 42}, []float64{153, -17, 303, 43}, []float64{152, -14, 302, 46}, []float64{152, -14, 302, 46}, []float64{152, -12, 302, 48}, []float64{152, -12, 302, 48}, []float64{152, -11, 302, 49}, []float64{152, -11, 302, 49}, []float64{152, -10, 302, 50}, []float64{152, -10, 302, 50}, []float64{152, -8, 302, 52}, []float64{152, -8, 302, 52}, []float64{151, -7, 301, 53}, []float64{151, -7, 301, 53}, []float64{151, -6, 301, 54}, []float64{151, -6, 301, 54}, []float64{151, -2, 301, 58}, []float64{150, 0, 300, 60}, []float64{151, 2, 301, 62}, []float64{151, 5, 301, 65}, []float64{151, 9, 301, 69}, []float64{150, 12, 300, 72}, []float64{150, 14, 300, 74}, []float64{148, 16, 298, 76}, []float64{147, 26, 297, 86}, []float64{148, 28, 298, 88}, []float64{148, 40, 298, 100}, []float64{148, 30, 298, 90}, []float64{147, 22, 297, 82}, []float64{147, 34, 297, 94}, []float64{147, 21, 297, 81}, []float64{148, 40, 298, 100}, []float64{147, 40, 297, 100}, []float64{147, 40, 297, 100}, []float64{147, 36, 297, 96}, []float64{147, 53, 297, 113}, []float64{147, 50, 297, 110}, []float64{148, 55, 298, 115}, []float64{147, 50, 297, 110}, []float64{149, 68, 299, 128}, []float64{146, 49, 296, 109}, []float64{147, 68, 297, 128}, []float64{146, 31, 296, 91}, []float64{147, 64, 297, 124}, []float64{148, 71, 298, 131}, []float64{146, 64, 296, 124}, []float64{146, 74, 296, 134}, []float64{146, 64, 296, 124}, []float64{145, 77, 295, 137}, []float64{147, 82, 297, 142}, []float64{147, 78, 297, 138}, []float64{147, 78, 297, 138}, []float64{146, 79, 296, 139}, []float64{146, 79, 296, 139}, []float64{146, 91, 296, 151}, []float64{147, 78, 297, 138}, []float64{147, 78, 297, 138}, []float64{148, 90, 298, 150}, []float64{147, 92, 297, 152}, []float64{147, 92, 297, 152}, []float64{148, 98, 298, 158}, []float64{147, 100, 297, 160}, []float64{146, 92, 296, 152}, []float64{148, 110, 298, 170}, []float64{149, 92, 299, 152}, []float64{149, 92, 299, 152}, []float64{149, 110, 299, 170}, []float64{149, 92, 299, 152}, []float64{148, 104, 298, 164}, []float64{149, 111, 299, 171}, []float64{149, 106, 299, 166}, []float64{149, 106, 299, 166}, []float64{148, 124, 298, 184}, []float64{151, 125, 301, 185}, []float64{151, 125, 301, 185}, []float64{147, 120, 297, 180}, []float64{151, 131, 301, 191}, []float64{147, 132, 297, 192}, []float64{153, 139, 303, 199}, []float64{151, 141, 301, 201}, []float64{147, 151, 297, 211}, []float64{151, 145, 301, 205}, []float64{151, 145, 301, 205}, []float64{146, 133, 296, 193}, []float64{152, 148, 302, 208}, []float64{152, 148, 302, 208}, []float64{152, 148, 302, 208}, []float64{152, 148, 302, 208}, []float64{152, 148, 302, 208}, []float64{152, 148, 302, 208}, []float64{152, 148, 302, 208}, []float64{152, 148, 302, 208}, []float64{152, 148, 302, 208}, []float64{152, 148, 302, 208}, []float64{152, 148, 302, 208}, []float64{152, 148, 302, 208}, []float64{152, 148, 302, 208}, []float64{152, 148, 302, 208}, []float64{152, 148, 302, 208}, []float64{152, 148, 302, 208}, []float64{152, 148, 302, 208}, []float64{152, 148, 302, 208}, []float64{152, 148, 302, 208}, []float64{152, 148, 302, 208}, []float64{152, 148, 302, 208}, []float64{152, 148, 302, 208}}
	tracker := NewNewSimpleTracker[*SimpleBlob](15.0, 5)
	dt := 1.0 / 25.0 // emulate 25 fps

	for idx := range bboxesOne {
		rectOne := NewRect(bboxesOne[idx][0], bboxesOne[idx][1], bboxesOne[idx][2]-bboxesOne[idx][0], bboxesOne[idx][3]-bboxesOne[idx][1])
		rectTwo := NewRect(bboxesTwo[idx][0], bboxesTwo[idx][1], bboxesTwo[idx][2]-bboxesTwo[idx][0], bboxesTwo[idx][3]-bboxesTwo[idx][1])
		rectThree := NewRect(bboxesThree[idx][0], bboxesThree[idx][1], bboxesThree[idx][2]-bboxesThree[idx][0], bboxesThree[idx][3]-bboxesThree[idx][1])

		blobOne := NewSimpleBlobWithTime(rectOne, dt)
		blobTwo := NewSimpleBlobWithTime(rectTwo, dt)
		blobThree := NewSimpleBlobWithTime(rectThree, dt)
		blobs := []*SimpleBlob{blobOne, blobTwo, blobThree}
		err := tracker.MatchObjects(blobs)
		if err != nil {
			t.Error(err)
			return
		}
	}

	correctNumOfObjects := 3
	numOfObjects := len(tracker.Objects)
	if numOfObjects != correctNumOfObjects {
		t.Errorf("incorrect number of objects: %d, expected: %d", numOfObjects, correctNumOfObjects)
		return
	}

	// Optional: CSV output
	file, err := os.Create("../data/blobs_similar.csv")
	if err != nil {
		t.Error(err)
		return
	}
	defer file.Close()

	writer := csv.NewWriter(file)
	defer writer.Flush()
	writer.Comma = ';'

	err = writer.Write([]string{"id", "track"})
	if err != nil {
		t.Error(err)
		return
	}

	for objectID, object := range tracker.Objects {
		track := object.GetTrack()
		data := make([]string, len(track))
		for idx, pt := range track {
			data[idx] = fmt.Sprintf("%f,%f", pt.X, pt.Y)
		}
		dataStr := strings.Join(data, "|")
		err = writer.Write([]string{objectID.String(), dataStr})
		if err != nil {
			t.Error(err)
			return
		}
	}
}
