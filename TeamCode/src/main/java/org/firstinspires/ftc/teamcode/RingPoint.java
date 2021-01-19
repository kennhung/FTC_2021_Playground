package org.firstinspires.ftc.teamcode;

import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Iterator;

public class RingPoint implements Comparable {
    private int x, y;
    private int index;

    public RingPoint(MatOfPoint matOfPoint, int index) {
        Rect r = Imgproc.boundingRect(matOfPoint);

        x = r.x + (r.width / 2);
        y = r.y + r.height;
        this.index = index;
    }

    public static ArrayList<RingPoint> toRingPointList(ArrayList<MatOfPoint> mop) {
        ArrayList<RingPoint> rps = new ArrayList<>();

        Iterator<MatOfPoint> it = mop.iterator();
        int i = 0;
        while (it.hasNext()) {
            MatOfPoint owo = it.next();
            rps.add(new RingPoint(owo, i));
            i++;
        }

        return rps;
    }

    public int getX() {
        return x;
    }

    public int getY() {
        return y;
    }

    public int getIndex() {
        return index;
    }

    @Override
    public int compareTo(Object o) {
        RingPoint rp = (RingPoint) o;

        if (Math.abs(this.y - rp.y) < 15) {
            int aDeltaX = Math.abs(this.x - 320);
            int bDeltaX = Math.abs(rp.x - 320);
            return aDeltaX - bDeltaX;
        }

        return -(this.y - rp.y);
    }
}
