package testDeplacement;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;

public class MatrixTransformations {

	private final static float CAMERA_FOCAL_LENGTH = 1;// 3.43f; // Exprimée en
														// px
														// private static Sensor
														// accelerometer;

	// private static Sensor magnetometer;
	// private static float[] mGravity;
	// private static float[] mGeomagnetic;

	public static void calcMatrix(Mat image, float roll, float pitch, float azimuth, MatOfPoint3f objectPoints, MatOfPoint2f imagePoints) throws Exception {

		// System.out.println("------------Object points-----------");
		// for (Point3 p3 : objectPoints.toList()) {
		// System.out.println(p3);
		// }
		// System.out.println("------------Image points-----------");
		// for (Point p2 : imagePoints.toList()) {
		// System.out.println(p2);
		// }
		if (image == null || objectPoints == null || imagePoints == null)
			throw new Exception("VNC ERROR : Null parameter in detection.");
		if (objectPoints.rows() != imagePoints.rows())
			throw new Exception("VNC ERROR : number of object points and image points are different.");

		// Récupération des paramètres intrinsèques à la caméra
		Mat cameraMatrix = buildIntrinsicParametersMatrix(image);

		// Construction d'une matrices représentant les coefficients de
		// distorsion de la caméra (aucune distorsion : matrice de zéros)
		MatOfDouble distCoeffs = new MatOfDouble(new Mat(1, 4, CvType.CV_64F));
		// distCoeffs.put(0, 0, 0);
		// distCoeffs.put(0, 1, 0);
		// distCoeffs.put(0, 2, 0);
		// distCoeffs.put(0, 3, 0);

		// Récupération de la matrice de correspondance 2D/3D
		Mat rvec = new Mat(); // Output de solvePnP
		Mat tvec = new Mat(); // Output de solvePnP
		// System.out.println("objectPoints : "+objectPoints.dump() );
		// System.out.println("imagePoints : "+imagePoints.dump() );
		// System.out.println("cameraMatrix : "+cameraMatrix.dump() );
		// System.out.println("distCoeffs : "+distCoeffs.dump() );
		Calib3d.solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);
		// System.out.println("rvec : "+rvec.dump() );
		// System.out.println("tvec : "+tvec.dump() );
		// Conversion vecteur de rotation vers matrice de rotation
		Mat rotationMatrix = new Mat(); // Output de Rodrigues
		Calib3d.Rodrigues(rvec, rotationMatrix);
		// System.out.println("rotationMatrix : "+rotationMatrix.dump() );
		// Construction de la matrice de passage
		Mat rtvec = new Mat(3, 4, CvType.CV_32F);
		rtvec.put(0, 0, rotationMatrix.get(0, 0)[0]);
		rtvec.put(0, 1, rotationMatrix.get(0, 1)[0]);
		rtvec.put(0, 2, rotationMatrix.get(0, 2)[0]);
		rtvec.put(0, 3, tvec.get(0, 0)[0]);
		rtvec.put(1, 0, rotationMatrix.get(1, 0)[0]);
		rtvec.put(1, 1, rotationMatrix.get(1, 1)[0]);
		rtvec.put(1, 2, rotationMatrix.get(1, 2)[0]);
		rtvec.put(1, 3, tvec.get(1, 0)[0]);
		rtvec.put(2, 0, rotationMatrix.get(2, 0)[0]);
		rtvec.put(2, 1, rotationMatrix.get(2, 1)[0]);
		rtvec.put(2, 2, rotationMatrix.get(2, 2)[0]);
		rtvec.put(2, 3, tvec.get(2, 0)[0]);

		// displayMatrix(rvec, "rvec");
		// displayMatrix(tvec, "tvec");
		// displayMatrix(rtvec, "rtvec");

		// // TODO : commentaire
		// Mat a = new Mat(3, 4, CvType.CV_32F); // Output de gemm
		// Core.gemm(cameraMatrix, rtvec, 1, new Mat(), 0, a, 0); // Equivalent
		// à : cameraMatrix * rtvec;

		// displayMatrix(cameraMatrix, "cameraMatrix");
		// displayMatrix(a, "a");

		// // TODO : commentaire
		// Mat b = new Mat(4, 1, CvType.CV_32F);
		// b.put(0, 0, 0);
		// b.put(1, 0, 0);
		// b.put(2, 0, 0);
		// b.put(3, 0, 1);
		//
		// // TODO : commentaire
		// Mat d = new Mat(3, 1, CvType.CV_32F); // Output de gemm
		// Core.gemm(a, b, 1, new Mat(), 0, d, 0);
		// double u = d.get(2, 0)[0];

		// // TODO : commentaire
		// Mat c = new Mat(3, 3, CvType.CV_32F);
		// c.put(0, 0, a.get(0, 0)[0]);
		// c.put(0, 1, a.get(0, 1)[0]);
		// c.put(0, 2, a.get(0, 3)[0]);
		// c.put(1, 0, a.get(1, 0)[0]);
		// c.put(1, 1, a.get(1, 1)[0]);
		// c.put(1, 2, a.get(1, 3)[0]);
		// c.put(2, 0, a.get(2, 0)[0]);
		// c.put(2, 1, a.get(2, 1)[0]);
		// c.put(2, 2, a.get(2, 3)[0]);

		// displayMatrix(c, "c");

		// Résolution du système matriciel : on récupère les coordonnées du
		// point touché (à l'écran) dans le repère du robot
		// Mat invC = c.inv();
		// touchedPointMatrix.put(2, 0, 1); // Ajout d'une coordonnée en Z pour
		// le
		// point touché (à l'écran), utile
		// seulement pour le calcul
		// Mat touchPointInRobotReference = new Mat(3, 1, CvType.CV_32F); //
		// Output de gemm
		// Core.gemm(invC, touchedPointMatrix, 1, new Mat(), 0,
		// touchPointInRobotReference, 0);

		// // Application du coeff u
		// touchPointInRobotReference.put(0, 0, u *
		// touchPointInRobotReference.get(0, 0)[0]);
		// touchPointInRobotReference.put(1, 0, u *
		// touchPointInRobotReference.get(1, 0)[0]);
		// touchPointInRobotReference.put(2, 0, u *
		// touchPointInRobotReference.get(2, 0)[0]);

		// ///////////////////////
		// APPLICATION DU CARSENAT
		// ///////////////////////

		// ///////////////////////////////////////////////

		// Initialisation du sensor manager
		// init(mSensorManager);

		// Initialisation des angles d'orientation de l'appareil
		// float pitch = 0.0f;

		// Récupération des paramètres d'orientation de l'appareil
		// if (mGravity != null && mGeomagnetic != null) {
		// float R[] = new float[9];
		// float I[] = new float[9];
		//
		// boolean success = SensorManager.getRotationMatrix(R, I, mGravity,
		// mGeomagnetic);
		// if (success) {
		// float orientation[] = new float[3];
		// SensorManager.getOrientation(R, orientation);
		//
		// // Pitch : angle selon x
		// orientation[2] = (float) (orientation[2] + Math.PI);
		// System.out.println("PITCH = " + orientation[2]);
		// pitch = (float) Math.toDegrees(orientation[2]);
		// System.out.println("PITCH = " + pitch);
		// }
		// }

		Mat cameraTranslationVector = new Mat(3, 1, CvType.CV_32F);
		Core.gemm(rotationMatrix.inv(), tvec, 1, new Mat(), 0, cameraTranslationVector, 0);
		// /////////////////////////////////////////////
		// System.out.println("ROLL = " + roll);
		roll = (float) (roll * Math.PI / 180.0);
		
		Mat MrcamX = new Mat(3, 3, CvType.CV_32F);
		MrcamX.put(0, 0, 1);
		MrcamX.put(0, 1, 0);
		MrcamX.put(0, 2, 0);
		MrcamX.put(1, 0, 0);
		MrcamX.put(1, 1, Math.cos(roll));
		MrcamX.put(1, 2, -Math.sin(roll));
		MrcamX.put(2, 0, 0);
		MrcamX.put(2, 1, Math.sin(roll));
		MrcamX.put(2, 2, Math.cos(roll));
		
//		pitch = (float) (pitch * Math.PI / 180.0);
//		
//		Mat MrcamY = new Mat(3, 3, CvType.CV_32F);
//		MrcamY.put(0, 0, Math.cos(pitch));
//		MrcamY.put(0, 1, 0);
//		MrcamY.put(0, 2, Math.sin(pitch));
//		MrcamY.put(1, 0, 0);
//		MrcamY.put(1, 1, 1);
//		MrcamY.put(1, 2, 0);
//		MrcamY.put(2, 0, -Math.sin(pitch));
//		MrcamY.put(2, 1, 0);
//		MrcamY.put(2, 2, Math.cos(pitch));
//		
//		Mat Mrcam = new Mat(3, 3, CvType.CV_32F);
//		Core.gemm(MrcamY, MrcamX, 1, new Mat(), 0, Mrcam, 0);
		Mat Mrcam = MrcamX;
		Mat Rcam = new Mat(); // Output de Rodrigues
		Calib3d.Rodrigues(Mrcam, Rcam);

		// displayMatrix(Rcam, "Rcam");
		// displayMatrix(MrcamX, "MrcamX");

		// System.out.println("rotationMatrix : "+rotationMatrix.dump() );
		Mat rotationMatrix32F = new Mat(3, 3, CvType.CV_32F);
		rotationMatrix32F.put(0, 0, rotationMatrix.get(0, 0)[0]);
		rotationMatrix32F.put(0, 1, rotationMatrix.get(0, 1)[0]);
		rotationMatrix32F.put(0, 2, rotationMatrix.get(0, 2)[0]);
		rotationMatrix32F.put(1, 0, rotationMatrix.get(1, 0)[0]);
		rotationMatrix32F.put(1, 1, rotationMatrix.get(1, 1)[0]);
		rotationMatrix32F.put(1, 2, rotationMatrix.get(1, 2)[0]);
		rotationMatrix32F.put(2, 0, rotationMatrix.get(2, 0)[0]);
		rotationMatrix32F.put(2, 1, rotationMatrix.get(2, 1)[0]);
		rotationMatrix32F.put(2, 2, rotationMatrix.get(2, 2)[0]);

		// displayMatrix(rotationMatrix32F, "rotationMatrix32F");

		Mat MrcamRotationX = new Mat(3, 3, CvType.CV_32F);
		Core.gemm(Mrcam, rotationMatrix32F, 1, new Mat(), 0, MrcamRotationX, 0);

		Mat Xrob = new Mat(3, 1, CvType.CV_32F);
		Mat unZeroZero = new Mat(3, 1, CvType.CV_32F);
		unZeroZero.put(0, 0, 1);
		unZeroZero.put(1, 0, 0);
		unZeroZero.put(2, 0, 0);
		Core.gemm(MrcamRotationX, unZeroZero, 1, new Mat(), 0, Xrob, 0);

		// displayMatrix(Xrob, "Xrob");

		Mat MRotation = new Mat(3, 3, CvType.CV_32F);

		double resMax = 0;
		int rotationRepereCamera = 0;
		Mat MRotationFinale = new Mat(3, 3, CvType.CV_32F);
		// System.out.println("rotationMatrix32F : "+rotationMatrix32F.dump() );
		for (int i = 0; i < 360; i += 5) {
			MRotation.put(0, 0, Math.cos(i * Math.PI / 180));
			MRotation.put(0, 1, -Math.sin(i * Math.PI / 180));
			MRotation.put(0, 2, 0);
			MRotation.put(1, 0, Math.sin(i * Math.PI / 180));
			MRotation.put(1, 1, Math.cos(i * Math.PI / 180));
			MRotation.put(1, 2, 0);
			MRotation.put(2, 0, 0);
			MRotation.put(2, 1, 0);
			MRotation.put(2, 2, 1);

			// System.out.println("MATRIX : i=" + i);
			// displayMatrix(MRotation, "MRotation");

			Mat Rrotation = new Mat(); // Output de Rodrigues
			Calib3d.Rodrigues(MRotation, Rrotation);

			Mat MrcamXrotationMatrix = new Mat(3, 3, CvType.CV_32F);
			Core.gemm(Mrcam, rotationMatrix32F, 1, new Mat(), 0, MrcamXrotationMatrix, 0);

			Mat MRotationMrcamXrotationMatrix = new Mat(3, 3, CvType.CV_32F);
			Core.gemm(MRotation, MrcamXrotationMatrix, 1, new Mat(), 0, MRotationMrcamXrotationMatrix, 0);

			Core.gemm(MRotationMrcamXrotationMatrix, unZeroZero, 1, new Mat(), 0, Xrob, 0);

			double res = Xrob.get(0, 0)[0];

			if (res > resMax) {
				resMax = res;
				rotationRepereCamera = i;
				MRotationFinale.put(0, 0, MRotation.get(0, 0)[0]);
				MRotationFinale.put(0, 1, MRotation.get(0, 1)[0]);
				MRotationFinale.put(0, 2, MRotation.get(0, 2)[0]);
				MRotationFinale.put(1, 0, MRotation.get(1, 0)[0]);
				MRotationFinale.put(1, 1, MRotation.get(1, 1)[0]);
				MRotationFinale.put(1, 2, MRotation.get(1, 2)[0]);
				MRotationFinale.put(2, 0, MRotation.get(2, 0)[0]);
				MRotationFinale.put(2, 1, MRotation.get(2, 1)[0]);
				MRotationFinale.put(2, 2, MRotation.get(2, 2)[0]);
			}
		}
		// displayMatrix(MRotationFinale, "MRotationFinale");
		// System.out.println("MRotationFinale : " + MRotationFinale.dump());
		Core.gemm(Mrcam, MRotationFinale, 1, new Mat(), 0, Mrcam, 0);

		// displayMatrix(MrcamX, "MrcamX");

		Calib3d.Rodrigues(Mrcam, Rcam);
		MatOfPoint3f zeroZeroZero = new MatOfPoint3f(new Point3(0, 0, 0));
		MatOfPoint2f centre = new MatOfPoint2f();
		Calib3d.projectPoints(zeroZeroZero, Rcam, tvec, cameraMatrix, distCoeffs, centre);

		Mat centerMatrix = new Mat(3, 1, CvType.CV_32F);
		centerMatrix.put(0, 0, centre.get(0, 0)[0]);
		centerMatrix.put(1, 0, centre.get(0, 0)[1]);
		centerMatrix.put(2, 0, 1);

		// displayMatrix(centerMatrix, "centerMatrix");

		Mat Ntvec = new Mat(3, 1, CvType.CV_32F);
		Core.gemm(cameraMatrix.inv(), centerMatrix, 1, new Mat(), 0, Ntvec, 0);
		// System.out.println("Ntvec first: " + Ntvec.dump());
		double indice = Math.sqrt(cameraTranslationVector.get(0, 0)[0] * cameraTranslationVector.get(0, 0)[0] + cameraTranslationVector.get(1, 0)[0] * cameraTranslationVector.get(1, 0)[0]
				+ cameraTranslationVector.get(2, 0)[0] * cameraTranslationVector.get(2, 0)[0]);
		// System.out.println("indice : " + indice);
		Core.multiply(Ntvec, new Scalar(indice), Ntvec);

		// displayMatrix(Ntvec, "Ntvec");
		// System.out.println("MATRIX : indice="+indice);

		// System.out.println("MrcamX : " + MrcamX.dump());
		// System.out.println("Ntvec : " + Ntvec.dump());

		Core.gemm(Mrcam.inv(), Ntvec, 1, new Mat(), 0, cameraTranslationVector, 0);

		// System.out.println("cameraTranslationVector : " +
		// cameraTranslationVector.dump());
		// displayMatrix(cameraTranslationVector, "cameraTranslationVector");

		// Mat coordonneesClique = new Mat(3, 1, CvType.CV_32F);
		// Core.gemm(cameraMatrix.inv(), touchedPointMatrix, 1, new Mat(), 0,
		// coordonneesClique, 0);

		// System.out.println("coordonneesClique : " +
		// coordonneesClique.dump());
		// displayMatrix(coordonneesClique, "coordonneesClique");

		// Core.gemm(MrcamX.inv(), coordonneesClique, 1, new Mat(), 0,
		// coordonneesClique, 0);

		// displayMatrix(coordonneesClique, "coordonneesClique FINAL");
		// System.out.println("coordonneesClique FINAL : " +
		// coordonneesClique.dump());
		// double t = -cameraTranslationVector.get(2, 0)[0] /
		// coordonneesClique.get(2, 0)[0];
		// double X = -coordonneesClique.get(0, 0)[0] * t -
		// cameraTranslationVector.get(0, 0)[0];
		// double Y = -coordonneesClique.get(1, 0)[0] * t -
		// cameraTranslationVector.get(1, 0)[0];

		MatOfPoint2f VX = new MatOfPoint2f();
		Calib3d.projectPoints(new MatOfPoint3f(new Point3(0, 0, 0), new Point3(0.58, 0, 0)), Rcam, Ntvec, cameraMatrix, distCoeffs, VX);
		MatOfPoint2f VY = new MatOfPoint2f();
		Calib3d.projectPoints(new MatOfPoint3f(new Point3(0, 0, 0), new Point3(0, 0.58, 0)), Rcam, Ntvec, cameraMatrix, distCoeffs, VY);
		MatOfPoint2f VZ = new MatOfPoint2f();
		Calib3d.projectPoints(new MatOfPoint3f(new Point3(0, 0, 0), new Point3(0, 0, 0.58)), Rcam, Ntvec, cameraMatrix, distCoeffs, VZ);
		Core.line(image, VX.toArray()[0], VX.toArray()[1], new Scalar(255, 0, 0), 2);
		Core.line(image, VY.toArray()[0], VY.toArray()[1], new Scalar(0, 255, 0), 2);
		Core.line(image, VZ.toArray()[0], VZ.toArray()[1], new Scalar(0, 0, 255), 2);

		MatOfPoint2f PointsCaptes = new MatOfPoint2f();
		Calib3d.projectPoints(objectPoints, rvec, tvec, cameraMatrix, distCoeffs, PointsCaptes);
		for (Point p : PointsCaptes.toArray()) {
			Core.circle(image, p, 20, new Scalar(255, 0, 0));
		}

		MatrixTransformations.MrcamX = Mrcam;
		MatrixTransformations.cameraMatrix = cameraMatrix;
		MatrixTransformations.cameraTranslationVector = cameraTranslationVector;
		// displayMatrix(cameraTranslationVector, "CAMERA TRANSLATION");
		// return new Point3(X, Y, 0);

	}

	private static Mat MrcamX;
	private static Mat cameraMatrix;
	private static Mat cameraTranslationVector;

	public static Point3 detect(Mat image, Point toDetect) throws Exception {
		if (image == null || toDetect == null)
			throw new Exception("VNC ERROR : Null parameter in detection.");
		if (MrcamX == null || cameraMatrix == null || cameraTranslationVector == null)
			throw new Exception("VNC ERROR : the matrix was not calculated before.");
		Mat coordonneesClique = new Mat(3, 1, CvType.CV_32F);
		Mat touchedPointMatrix = new Mat(3, 1, CvType.CV_32F);
		touchedPointMatrix.put(0, 0, toDetect.x);
		touchedPointMatrix.put(1, 0, toDetect.y);
		touchedPointMatrix.put(2, 0, 1);
		Core.gemm(cameraMatrix.inv(), touchedPointMatrix, 1, new Mat(), 0, coordonneesClique, 0);
		Core.gemm(MrcamX.inv(), coordonneesClique, 1, new Mat(), 0, coordonneesClique, 0);
		double t = -cameraTranslationVector.get(2, 0)[0] / coordonneesClique.get(2, 0)[0];
		double X = -coordonneesClique.get(0, 0)[0] * t - cameraTranslationVector.get(0, 0)[0];
		double Y = -coordonneesClique.get(1, 0)[0] * t - cameraTranslationVector.get(1, 0)[0];

		return new Point3(X, Y, 0);

	}

	// public static double pixelToMeter(DisplayMetrics displayMetrics, double
	// pixelValue) {
	//
	// return (pixelValue / displayMetrics.xdpi * 25.4) / 1000;
	// }

	protected static Mat buildIntrinsicParametersMatrix(Mat image) {

		// Récupération de la focale en pixels
		// TODO : float focalLength = cameraParameters.getFocalLength();
		// float focalLengthInPixel =
		// TypedValue.applyDimension(TypedValue.COMPLEX_UNIT_MM,
		// CAMERA_FOCAL_LENGTH*1000, displayMetrics);

		// Coordonnées du centre de l'écran dans notre repère
		float centreX = (float) ((image.width() - 1) / 2.0f);
		float centreY = (float) ((image.height() - 1) / 2.0f);

		// Initialisation de la matrice des paramètres intrinsèques à la caméra
		// et ajout des composants à la matrice
		Mat intrinsicParametersMatrix = new Mat(3, 3, CvType.CV_32F);
		intrinsicParametersMatrix.put(0, 0, CAMERA_FOCAL_LENGTH * image.width());
		intrinsicParametersMatrix.put(0, 1, 0);
		intrinsicParametersMatrix.put(0, 2, centreX);
		intrinsicParametersMatrix.put(1, 0, 0);
		intrinsicParametersMatrix.put(1, 1, CAMERA_FOCAL_LENGTH * image.width());
		intrinsicParametersMatrix.put(1, 2, centreY);
		intrinsicParametersMatrix.put(2, 0, 0);
		intrinsicParametersMatrix.put(2, 1, 0);
		intrinsicParametersMatrix.put(2, 2, 1);

		return intrinsicParametersMatrix;

	}

	// protected Mat buildExtrinsicRotationParametersMatrix(SensorManager
	// mSensorManager) {
	//
	// // Initialisation du sensor manager
	// init(mSensorManager);
	//
	// // Initialisation des angles d'orientation de l'appareil
	// float pitch = 0.0f;
	// float roll = 0.0f;
	// float azimuth = 0.0f;
	//
	// // Récupération des paramètres d'orientation de l'appareil
	// if (mGravity != null && mGeomagnetic != null) {
	// float R[] = new float[9];
	// float I[] = new float[9];
	//
	// boolean success = SensorManager.getRotationMatrix(R, I, mGravity,
	// mGeomagnetic);
	// if (success) {
	// float orientation[] = new float[3];
	// SensorManager.getOrientation(R, orientation);
	//
	// // Pitch : angle selon x
	// pitch = (float) Math.toDegrees(orientation[1]);
	// // Roll : angle selon y
	// roll = (float) Math.toDegrees(orientation[2]);
	// // Azimuth : angle selon z
	// azimuth = (float) Math.toDegrees(orientation[0]);
	// }
	// }
	//
	// // Initialisation de la matrice des paramètres intrinsèques à la caméra
	// // et ajout des composants à la matrice
	// Mat extrinsicRotationParametersMatrix = new Mat(3, 3, CvType.CV_32F);
	// extrinsicRotationParametersMatrix.put(0, 0, Math.cos(azimuth) *
	// Math.cos(roll));
	// extrinsicRotationParametersMatrix.put(0, 1, Math.cos(azimuth) *
	// Math.sin(roll) * Math.sin(pitch) - Math.sin(azimuth) * Math.cos(pitch));
	// extrinsicRotationParametersMatrix.put(0, 2, Math.cos(azimuth) *
	// Math.sin(roll) * Math.cos(pitch) + Math.sin(azimuth) * Math.sin(pitch));
	// extrinsicRotationParametersMatrix.put(1, 0, Math.sin(azimuth) *
	// Math.cos(roll));
	// extrinsicRotationParametersMatrix.put(1, 1, Math.sin(azimuth) *
	// Math.sin(roll) * Math.sin(pitch) + Math.cos(azimuth) * Math.cos(pitch));
	// extrinsicRotationParametersMatrix.put(1, 2, Math.sin(azimuth) *
	// Math.sin(roll) * Math.cos(pitch) - Math.cos(azimuth) * Math.sin(pitch));
	// extrinsicRotationParametersMatrix.put(2, 0, -Math.sin(roll));
	// extrinsicRotationParametersMatrix.put(2, 1, Math.cos(roll) *
	// Math.sin(pitch));
	// extrinsicRotationParametersMatrix.put(2, 2, Math.cos(roll) *
	// Math.cos(pitch));
	//
	// return extrinsicRotationParametersMatrix;
	// }
	//
	// private static void init(SensorManager mSensorManager) {
	// setAccelerometer(mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER));
	// setMagnetometer(mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD));
	//
	// // Listen
	// mSensorManager.registerListener(sel, accelerometer,
	// SensorManager.SENSOR_DELAY_NORMAL);
	// mSensorManager.registerListener(sel, magnetometer,
	// SensorManager.SENSOR_DELAY_NORMAL);
	// }
	//
	// private static SensorEventListener sel = new SensorEventListener() {
	//
	// @Override
	// public void onAccuracyChanged(Sensor sensor, int accuracy) {
	// // System.out.println("VNCTests : onAccuracyChanged");
	// }
	//
	// @Override
	// public void onSensorChanged(SensorEvent event) {
	// // System.out.println("VNCTests : onSensorChanged");
	// if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
	// mGravity = event.values;
	// // System.out.println("VNCTests : Orientation mGravity = " +
	// // mGravity);
	// }
	// if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
	// mGeomagnetic = event.values;
	// // System.out.println("VNCTests : Orientation mGeomagnetic = " +
	// // mGeomagnetic);
	// }
	// }
	// };

	public static void displayMatrix(Mat matrix, String name) {
		System.out.println("MATRIX : " + name);
		String line = "";
		for (int i = 0; i < matrix.rows(); i++) {
			for (int j = 0; j < matrix.cols(); j++) {
				line += matrix.get(i, j)[0] + " ";
			}
			System.out.println("MATRIX : " + line);
			line = "";
		}
	}

	// public Sensor getAccelerometer() {
	// return accelerometer;
	// }
	//
	// public Sensor getMagnetometer() {
	// return magnetometer;
	// }
	//
	// private static void setMagnetometer(Sensor defaultSensor) {
	// magnetometer = defaultSensor;
	//
	// }
	//
	// private static void setAccelerometer(Sensor defaultSensor) {
	// accelerometer = defaultSensor;
	// }
}