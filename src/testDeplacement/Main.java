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

	public static void loadTest1() {
		Main.image = Highgui.imread("img/Screenshot_2014-12-09-14-27-21.png");
		Main.roll = 107.5f;
		final Point3 headP3 = new Point3(-0.01538732647895813, -0.0183858722448349, 0.4551582932472229);
		final Point headP2 = new Point(525, 239);
		final Point3 handLeftP3 = new Point3(0.032784223556518555, 0.10014607757329941, 0.2254450023174286);
		final Point handLeftP2 = new Point(624, 490);
		final Point3 handRightP3 = new Point3(0.0391731858253479, -0.11687855422496796, 0.21903225779533386);
		final Point handRightP2 = new Point(436, 490);
		final Point3 shoulderLeftP3 = new Point3(-0.016333427280187607, 0.07193655520677567, 0.43833619356155396);
		final Point shoulderLeftP2 = new Point(597, 286);
		final Point3 feetsMiddleP3 = new Point3(0, 0, 0);
		final Point feetsMiddleP2 = new Point(536, 596);
		Main.objectPoints = new MatOfPoint3f(headP3, handLeftP3, handRightP3, shoulderLeftP3, feetsMiddleP3);
		Main.imagePoints = new MatOfPoint2f(headP2, handLeftP2, handRightP2, shoulderLeftP2, feetsMiddleP2);
	}

	public static void loadTest2() {
		Main.image = Highgui.imread("img/IMG_20141017_154109.jpg");
		Main.roll = 105f;
		final Point3 headP3 = new Point3(-0.01538732647895813, -0.0183858722448349, 0.4551582932472229);
		final Point headP2 = new Point(1813, 721);
		final Point3 handLeftP3 = new Point3(0.032784223556518555, 0.10014607757329941, 0.2254450023174286);
		final Point handLeftP2 = new Point(2209, 1525);
		final Point3 handRightP3 = new Point3(0.0391731858253479, -0.11687855422496796, 0.21903225779533386);
		final Point handRightP2 = new Point(1389, 1525);
		final Point3 shoulderLeftP3 = new Point3(-0.016333427280187607, 0.07193655520677567, 0.43833619356155396);
		final Point shoulderLeftP2 = new Point(2109, 917);
		final Point3 feetsMiddleP3 = new Point3(0, 0, 0);
		final Point feetsMiddleP2 = new Point(1777, 2129);
		Main.objectPoints = new MatOfPoint3f(headP3, handLeftP3, handRightP3, shoulderLeftP3, feetsMiddleP3);
		Main.imagePoints = new MatOfPoint2f(headP2, handLeftP2, handRightP2, shoulderLeftP2, feetsMiddleP2);
	}

	public static void loadTest3() {
		Main.image = Highgui.imread("img/20120114_060107.jpg");
		Main.roll = 110f;
		final Point3 headP3 = new Point3(-0.01538732647895813, -0.0183858722448349, 0.4551582932472229);
		final Point headP2 = new Point(1675, 439);
		final Point3 handLeftP3 = new Point3(0.032784223556518555, 0.10014607757329941, 0.2254450023174286);
		final Point handLeftP2 = new Point(2000, 1117);
		final Point3 handRightP3 = new Point3(0.0391731858253479, -0.11687855422496796, 0.21903225779533386);
		final Point handRightP2 = new Point(1331, 1117);
		final Point3 shoulderLeftP3 = new Point3(-0.016333427280187607, 0.07193655520677567, 0.43833619356155396);
		final Point shoulderLeftP2 = new Point(1909, 600);
		final Point3 feetsMiddleP3 = new Point3(0, 0, 0);
		final Point feetsMiddleP2 = new Point(1641, 1621);
		Main.objectPoints = new MatOfPoint3f(headP3, handLeftP3, handRightP3, shoulderLeftP3, feetsMiddleP3);
		Main.imagePoints = new MatOfPoint2f(headP2, handLeftP2, handRightP2, shoulderLeftP2, feetsMiddleP2);
	}

	public static void loadTest4() {
		Main.image = Highgui.imread("img/Screenshot_2014-12-15-11-13-40.png");
		Main.roll = 108.43f - 180; // TODO : Mickael envoie du roll + 180
		Main.pitch = 179.1f - 180; // TODO : Mickael envoie du pitch + 180
		Main.azimuth = 133.87f - 180; // TODO : Mickael envoie du azimuth + 180
		final Point3 headP3 = new Point3(-0.009586147964000702, 0.03390171378850937, 0.4526781737804413);
		final Point headP2 = new Point(639, 207);
		final Point3 handLeftP3 = new Point3(0.0346740186214447, 0.13087007403373718, 0.20899218320846558);
		final Point handLeftP2 = new Point(713, 373);
		final Point3 handRightP3 = new Point3(0.04136417806148529, -0.09935460984706879, 0.2295495569705963);
		final Point handRightP2 = new Point(564, 367);
		final Point3 footLeftP3 = new Point3(-0.005090042948722839, 0.06007814407348633, -0.00953608751296997);
		final Point footLeftP2 = new Point(674, 471);
		final Point3 footRightP3 = new Point3(-0.0029234662652015686, -0.06019705533981323, -1.0371208190917969E-5);
		final Point footRightP2 = new Point(609, 472);
		Main.objectPoints = new MatOfPoint3f(headP3, handLeftP3, handRightP3, footLeftP3, footRightP3);
		Main.imagePoints = new MatOfPoint2f(headP2, handLeftP2, handRightP2, footLeftP2, footRightP2);
	}

	public static void loadTest5() {

		Main.image = Highgui.imread("img/Screenshot_2015-01-07-09-59-26.png");

		Main.roll = -65f;
		Main.pitch = 11f;

		final Point3 headP3 = new Point3(-0.009586147964000702, 0.03390171378850937, 0.4526781737804413);
		final Point3 handLeftP3 = new Point3(0.0346740186214447, 0.13087007403373718, 0.20899218320846558);
		final Point3 handRightP3 = new Point3(0.04136417806148529, -0.09935460984706879, 0.2295495569705963);
		final Point3 shoulderLeftP3 = new Point3(-0.011, 0.106, 0.43);
		final Point3 footRightP3 = new Point3(-0.0029234662652015686, -0.06019705533981323, -1.0371208190917969E-5);
		Main.objectPoints = new MatOfPoint3f(headP3, handLeftP3, handRightP3, shoulderLeftP3, footRightP3);

		final Point headP2 = new Point(705, 166);
		final Point handLeftP2 = new Point(741, 348);
		final Point handRightP2 = new Point(600, 307);
		final Point shoulderLeftP2 = new Point(686, 290);
		final Point footRightP2 = new Point(609, 432);
		Main.imagePoints = new MatOfPoint2f(headP2, handLeftP2, handRightP2, shoulderLeftP2, footRightP2);
	}

	// PRECISION ++
	
	public static void loadTest6() {

		Main.image = Highgui.imread("img/Screenshot_2015-01-07-10-17-57.png");

		Main.roll = -68.89746f;
		Main.pitch = 12.903547f;
		final Point p2D0 = new Point(621.0, 186.0);
		final Point3 p3D0 = new Point3(-0.017337866127490997, 0.017378125339746475, 0.455221563577652);
		final Point p2D1 = new Point(497.0, 350.0);
		final Point3 p3D1 = new Point3(0.03804643452167511, -0.10589829087257385, 0.22758351266384125);
		final Point p2D2 = new Point(666.0, 399.0);
		final Point3 p3D2 = new Point3(0.0327814519405365, 0.1229027733206749, 0.21722161769866943);
		final Point p2D3 = new Point(552.0, 214.0);
		final Point3 p3D3 = new Point3(-0.017007561400532722, -0.07358086109161377, 0.4381466805934906);
		final Point p2D4 = new Point(510.0, 495.2);
		final Point3 p3D4 = new Point3(-0.003983132541179657, -0.06754104048013687, -1.004338264465332E-5);
		Main.objectPoints = new MatOfPoint3f(p3D0, p3D1, p3D2, p3D3, p3D4);
		Main.imagePoints = new MatOfPoint2f(p2D0, p2D1, p2D2, p2D3, p2D4);
	}

	public static void loadTest7() {

		Main.image = Highgui.imread("img/Screenshot_2015-01-07-10-23-33.png");

		Main.roll = -66.04201f;
		Main.pitch = -16.043692f;
		final Point p2D0 = new Point(795.0, 130.0);
		final Point3 p3D0 = new Point3(-0.0175936296582222, 0.017584308981895447, 0.45519179105758667);
		final Point p2D1 = new Point(755.0, 342.0);
		final Point3 p3D1 = new Point3(0.037866655737161636, -0.10585038363933563, 0.22758346796035767);
		final Point p2D2 = new Point(939.0, 316.0);
		final Point3 p3D2 = new Point3(0.1174665, 0.11092513799, 0.2254450023174286);
		final Point p2D3 = new Point(789.0, 118.0);
		final Point3 p3D3 = new Point3(-0.01734914258122444, -0.07330982387065887, 0.43815624713897705);
		final Point p2D4 = new Point(821.0, 474.8);
		final Point3 p3D4 = new Point3(-0.004014141857624054, -0.06768251955509186, 8.106231689453125E-6);
		final Point p2D5 = new Point(921.0, 458.4);
		final Point3 p3D5 = new Point3(-0.00503096729516983, 0.06761455535888672, -0.005265861749649048);
		final Point p2D6 = new Point(871.0, 466.6);
		final Point3 p3D6 = new Point3(0.0, 0.0, 0.0);
		Main.objectPoints = new MatOfPoint3f(p3D0, p3D1, p3D2, p3D3, p3D4, p3D5, p3D6);
		Main.imagePoints = new MatOfPoint2f(p2D0, p2D1, p2D2, p2D3, p2D4, p2D5, p2D6);
	}

	// NEW VERSION 1
	
	public static void loadTest8() {

		Main.image = Highgui.imread("img/.png");

		
	}
	
	public static void loadTest9() {

		Main.image = Highgui.imread("img/.png");

		
	}
	
	public static void loadTest10() {

		Main.image = Highgui.imread("img/.png");

		
	}
	
	public static void main(String[] args) {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

		Main.loadTest4();
		try {
			MatrixTransformations.calcMatrix(Main.image, Main.roll, Main.pitch, Main.azimuth, Main.objectPoints, Main.imagePoints);

			Main.backgroundImage = Main.toBufferedImage(Main.image);
			try {
				final Frame frame = new JFrame("Test deplacement") {

					/**
					 *
					 */
					private static final long serialVersionUID = 544714257895018142L;

					@Override
					public void paint(Graphics g) {
						super.paint(g);
						if (Main.backgroundImage != null)
							g.drawImage(Main.backgroundImage, 0, 0, null);
					}
				};
				frame.setSize(Main.image.width(), Main.image.height());
				frame.setVisible(true);
				frame.addMouseListener(new MouseListener() {

					@Override
					public void mouseClicked(MouseEvent e) {
						try {
							// Mat touchedPointMatrix = new Mat(3, 1,
							// CvType.CV_32F);
							// touchedPointMatrix.put(0, 0, e.getX());
							// touchedPointMatrix.put(1, 0, e.getY());
							final Point3 res = MatrixTransformations.detect(new Point(e.getX(), e.getY()));
							System.out.println("{X=" + e.getX() + ",Y=" + e.getY() + " => " + res);
							Core.circle(Main.image, new Point(e.getX(), e.getY()), 20, new Scalar(255, 255, 255));
							Main.backgroundImage = Main.toBufferedImage(Main.image);
							frame.repaint();
						} catch (final Exception e1) {
							// TODO Auto-generated catch block
							e1.printStackTrace();
						}

					}

					@Override
					public void mouseEntered(MouseEvent e) {
						// TODO Auto-generated method stub

					}

					@Override
					public void mouseExited(MouseEvent e) {
						// TODO Auto-generated method stub

					}

					@Override
					public void mousePressed(MouseEvent e) {
						// TODO Auto-generated method stub

					}

					@Override
					public void mouseReleased(MouseEvent e) {
						// TODO Auto-generated method stub

					}
				});
			} catch (final HeadlessException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		} catch (final Exception e2) {
			// TODO Auto-generated catch block
			e2.printStackTrace();
		}
	}

	public static Image toBufferedImage(Mat m) {
		int type = BufferedImage.TYPE_BYTE_GRAY;
		if (m.channels() > 1)
			type = BufferedImage.TYPE_3BYTE_BGR;
		final int bufferSize = m.channels() * m.cols() * m.rows();
		final byte[] b = new byte[bufferSize];
		m.get(0, 0, b); // get all the pixels
		final BufferedImage image = new BufferedImage(m.cols(), m.rows(), type);
		final byte[] targetPixels = ((DataBufferByte) image.getRaster().getDataBuffer()).getData();
		System.arraycopy(b, 0, targetPixels, 0, b.length);
		return image;

	}

	private static Image backgroundImage = null;

	private static Mat image;

	private static MatOfPoint3f objectPoints;

	private static MatOfPoint2f imagePoints;

	private static float roll, pitch, azimuth;

}
