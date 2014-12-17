package testDeplacement;

import java.awt.Frame;
import java.awt.Graphics;
import java.awt.HeadlessException;
import java.awt.Image;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;

import javax.swing.JFrame;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.highgui.Highgui;

public class Main {

	private static Image backgroundImage = null;
	private static Mat image;
	private static MatOfPoint3f objectPoints;
	private static MatOfPoint2f imagePoints;
	private static float roll, pitch, azimuth;

	public static void loadTest1() {
		image = Highgui.imread("img/Screenshot_2014-12-09-14-27-21.png");
		roll = 107.5f;
		Point3 headP3 = new Point3(-0.01538732647895813, -0.0183858722448349, 0.4551582932472229);
		Point headP2 = new Point(525, 239);
		Point3 handLeftP3 = new Point3(0.032784223556518555, 0.10014607757329941, 0.2254450023174286);
		Point handLeftP2 = new Point(624, 490);
		Point3 handRightP3 = new Point3(0.0391731858253479, -0.11687855422496796, 0.21903225779533386);
		Point handRightP2 = new Point(436, 490);
		Point3 shoulderLeftP3 = new Point3(-0.016333427280187607, 0.07193655520677567, 0.43833619356155396);
		Point shoulderLeftP2 = new Point(597, 286);
		Point3 feetsMiddleP3 = new Point3(0, 0, 0);
		Point feetsMiddleP2 = new Point(536, 596);
		objectPoints = new MatOfPoint3f(headP3, handLeftP3, handRightP3, shoulderLeftP3, feetsMiddleP3);
		imagePoints = new MatOfPoint2f(headP2, handLeftP2, handRightP2, shoulderLeftP2, feetsMiddleP2);
	}

	public static void loadTest2() {
		image = Highgui.imread("img/IMG_20141017_154109.jpg");
		roll = 105f;
		Point3 headP3 = new Point3(-0.01538732647895813, -0.0183858722448349, 0.4551582932472229);
		Point headP2 = new Point(1813, 721);
		Point3 handLeftP3 = new Point3(0.032784223556518555, 0.10014607757329941, 0.2254450023174286);
		Point handLeftP2 = new Point(2209, 1525);
		Point3 handRightP3 = new Point3(0.0391731858253479, -0.11687855422496796, 0.21903225779533386);
		Point handRightP2 = new Point(1389, 1525);
		Point3 shoulderLeftP3 = new Point3(-0.016333427280187607, 0.07193655520677567, 0.43833619356155396);
		Point shoulderLeftP2 = new Point(2109, 917);
		Point3 feetsMiddleP3 = new Point3(0, 0, 0);
		Point feetsMiddleP2 = new Point(1777, 2129);
		objectPoints = new MatOfPoint3f(headP3, handLeftP3, handRightP3, shoulderLeftP3, feetsMiddleP3);
		imagePoints = new MatOfPoint2f(headP2, handLeftP2, handRightP2, shoulderLeftP2, feetsMiddleP2);
	}

	public static void loadTest3() {
		image = Highgui.imread("img/20120114_060107.jpg");
		roll = 110f;
		Point3 headP3 = new Point3(-0.01538732647895813, -0.0183858722448349, 0.4551582932472229);
		Point headP2 = new Point(1675, 439);
		Point3 handLeftP3 = new Point3(0.032784223556518555, 0.10014607757329941, 0.2254450023174286);
		Point handLeftP2 = new Point(2000, 1117);
		Point3 handRightP3 = new Point3(0.0391731858253479, -0.11687855422496796, 0.21903225779533386);
		Point handRightP2 = new Point(1331, 1117);
		Point3 shoulderLeftP3 = new Point3(-0.016333427280187607, 0.07193655520677567, 0.43833619356155396);
		Point shoulderLeftP2 = new Point(1909, 600);
		Point3 feetsMiddleP3 = new Point3(0, 0, 0);
		Point feetsMiddleP2 = new Point(1641, 1621);
		objectPoints = new MatOfPoint3f(headP3, handLeftP3, handRightP3, shoulderLeftP3, feetsMiddleP3);
		imagePoints = new MatOfPoint2f(headP2, handLeftP2, handRightP2, shoulderLeftP2, feetsMiddleP2);
	}

	public static void loadTest4() {
		image = Highgui.imread("img/Screenshot_2014-12-15-11-13-40.png");
		roll = 108.43f;
		pitch = 179.1f;
		azimuth = 133.87f;
		Point3 headP3 = new Point3(-0.009586147964000702, 0.03390171378850937, 0.4526781737804413);
		Point headP2 = new Point(639, 207);
		Point3 handLeftP3 = new Point3(0.0346740186214447, 0.13087007403373718, 0.20899218320846558);
		Point handLeftP2 = new Point(713, 373);
		Point3 handRightP3 = new Point3(0.04136417806148529, -0.09935460984706879, 0.2295495569705963);
		Point handRightP2 = new Point(564, 367);
		Point3 footLeftP3 = new Point3(-0.005090042948722839, 0.06007814407348633, -0.00953608751296997);
		Point footLeftP2 = new Point(674, 471);
		Point3 footRightP3 = new Point3(-0.0029234662652015686, -0.06019705533981323, -1.0371208190917969E-5);
		Point footRightP2 = new Point(609, 472);
		objectPoints = new MatOfPoint3f(headP3, handLeftP3, handRightP3, footLeftP3, footRightP3);
		imagePoints = new MatOfPoint2f(headP2, handLeftP2, handRightP2, footLeftP2, footRightP2);
	}

	public static void main(String[] args) {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

		loadTest4();
		try {
			MatrixTransformations.calcMatrix(image, roll, pitch, azimuth, objectPoints, imagePoints);

			backgroundImage = toBufferedImage(image);
			try {
				Frame frame = new JFrame("Test deplacement") {

					public void paint(Graphics g) {
						super.paint(g);
						if (backgroundImage != null)
							g.drawImage(backgroundImage, 0, 0, null);
					}
				};
				frame.setSize(image.width(), image.height());
				frame.setVisible(true);
				frame.addMouseListener(new MouseListener() {

					@Override
					public void mouseReleased(MouseEvent e) {
						// TODO Auto-generated method stub

					}

					@Override
					public void mousePressed(MouseEvent e) {
						// TODO Auto-generated method stub

					}

					@Override
					public void mouseExited(MouseEvent e) {
						// TODO Auto-generated method stub

					}

					@Override
					public void mouseEntered(MouseEvent e) {
						// TODO Auto-generated method stub

					}

					@Override
					public void mouseClicked(MouseEvent e) {
						try {
							// Mat touchedPointMatrix = new Mat(3, 1,
							// CvType.CV_32F);
							// touchedPointMatrix.put(0, 0, e.getX());
							// touchedPointMatrix.put(1, 0, e.getY());
							Point3 res = MatrixTransformations.detect(image, new Point(e.getX(), e.getY()));
							System.out.println("{X=" + e.getX() + ",Y=" + e.getY() + " => " + res);
							Core.circle(image, new Point(e.getX(), e.getY()), 20, new Scalar(255, 255, 255));
							backgroundImage = toBufferedImage(image);
							frame.repaint();
						} catch (Exception e1) {
							// TODO Auto-generated catch block
							e1.printStackTrace();
						}

					}
				});
			} catch (HeadlessException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		} catch (Exception e2) {
			// TODO Auto-generated catch block
			e2.printStackTrace();
		}
	}

	public static Image toBufferedImage(Mat m) {
		int type = BufferedImage.TYPE_BYTE_GRAY;
		if (m.channels() > 1) {
			type = BufferedImage.TYPE_3BYTE_BGR;
		}
		int bufferSize = m.channels() * m.cols() * m.rows();
		byte[] b = new byte[bufferSize];
		m.get(0, 0, b); // get all the pixels
		BufferedImage image = new BufferedImage(m.cols(), m.rows(), type);
		final byte[] targetPixels = ((DataBufferByte) image.getRaster().getDataBuffer()).getData();
		System.arraycopy(b, 0, targetPixels, 0, b.length);
		return image;

	}

}
