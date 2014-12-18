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

	private final static float CAMERA_FOCAL_LENGTH = 1; // 3.43f; // Exprimée en px

	public static void calcMatrix(Mat image, float roll, float pitch, float azimuth, MatOfPoint3f objectPoints, MatOfPoint2f imagePoints) throws Exception {

		// Détection d'éventuelles données importantes manquantes
		if (image == null || objectPoints == null || imagePoints == null)
			throw new Exception("VNC ERROR : Null parameter in detection.");
		if (objectPoints.rows() != imagePoints.rows())
			throw new Exception("VNC ERROR : number of object points and image points are different.");

		// Récupération des paramètres intrinsèques à la caméra
		Mat intrinsicParametersMatrix = buildIntrinsicParametersMatrix(image);

		// Construction d'une matrices représentant les coefficients de distorsion de la caméra (aucune distorsion : matrice de zéros)
		MatOfDouble distortionCoefficients = new MatOfDouble(new Mat(1, 4, CvType.CV_64F));

		// Récupération de la matrice de correspondance 2D/3D
		Mat rotationVector = new Mat(); // Output de solvePnP
		Mat translationVector = new Mat(); // Output de solvePnP
		Calib3d.solvePnP(objectPoints, imagePoints, intrinsicParametersMatrix, distortionCoefficients, rotationVector, translationVector);

		// Conversion vecteur de rotation vers matrice de rotation
		Mat rotationMatrix = new Mat(); // Output de Rodrigues
		Calib3d.Rodrigues(rotationVector, rotationMatrix);

		// Construction de la matrice de passage
		Mat cameraTranslationVector = new Mat(3, 1, CvType.CV_32F);
		Core.gemm(rotationMatrix.inv(), translationVector, 1, new Mat(), 0, cameraTranslationVector, 0);

		// Construction de la matrice de rotation qui correspond à la rotation que fait l'appareil autours de son axe x
		roll = degreeToRadian(roll);
		Mat cameraRotationMatrixAroundX = new Mat(3, 3, CvType.CV_32F);
		cameraRotationMatrixAroundX.put(0, 0, 1);
		cameraRotationMatrixAroundX.put(0, 1, 0);
		cameraRotationMatrixAroundX.put(0, 2, 0);
		cameraRotationMatrixAroundX.put(1, 0, 0);
		cameraRotationMatrixAroundX.put(1, 1, Math.cos(roll));
		cameraRotationMatrixAroundX.put(1, 2, -Math.sin(roll));
		cameraRotationMatrixAroundX.put(2, 0, 0);
		cameraRotationMatrixAroundX.put(2, 1, Math.sin(roll));
		cameraRotationMatrixAroundX.put(2, 2, Math.cos(roll));

		// Construction de la matrice de rotation qui correspond à la rotation que fait l'appareil autours de son axe y
		pitch = degreeToRadian(pitch);
		Mat cameraRotationMatrixAroundY = new Mat(3, 3, CvType.CV_32F);
		cameraRotationMatrixAroundY.put(0, 0, Math.cos(pitch));
		cameraRotationMatrixAroundY.put(0, 1, 0);
		cameraRotationMatrixAroundY.put(0, 2, Math.sin(pitch));
		cameraRotationMatrixAroundY.put(1, 0, 0);
		cameraRotationMatrixAroundY.put(1, 1, 1);
		cameraRotationMatrixAroundY.put(1, 2, 0);
		cameraRotationMatrixAroundY.put(2, 0, -Math.sin(pitch));
		cameraRotationMatrixAroundY.put(2, 1, 0);
		cameraRotationMatrixAroundY.put(2, 2, Math.cos(pitch));

		// Calcul de la matrice de rotation de l'appareil
		Mat cameraRotationMatrix = new Mat(3, 3, CvType.CV_32F);
		Core.gemm(cameraRotationMatrixAroundX, cameraRotationMatrixAroundY, 1, new Mat(), 0, cameraRotationMatrix, 0);

		// Rotation du repère appareil dans celui d ela caméra openCV (-PI/2 selon l'axe x)
		cameraRotationMatrix = matrixRotationFromX(cameraRotationMatrix, Math.PI);

		Mat Rcam = new Mat(); // Output de Rodrigues
		Calib3d.Rodrigues(cameraRotationMatrix, Rcam);

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

		Mat MrcamRotationX = new Mat(3, 3, CvType.CV_32F);
		Core.gemm(cameraRotationMatrix, rotationMatrix32F, 1, new Mat(), 0, MrcamRotationX, 0);

		Mat Xrob = new Mat(3, 1, CvType.CV_32F);
		Mat unZeroZero = new Mat(3, 1, CvType.CV_32F);
		unZeroZero.put(0, 0, 1);
		unZeroZero.put(1, 0, 0);
		unZeroZero.put(2, 0, 0);
		Core.gemm(MrcamRotationX, unZeroZero, 1, new Mat(), 0, Xrob, 0);

		Mat MRotation = new Mat(3, 3, CvType.CV_32F);

		double resMax = 0;
		int rotationRepereCamera = 0;
		Mat MRotationFinale = new Mat(3, 3, CvType.CV_32F);
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

			Mat Rrotation = new Mat(); // Output de Rodrigues
			Calib3d.Rodrigues(MRotation, Rrotation);

			Mat MrcamXrotationMatrix = new Mat(3, 3, CvType.CV_32F);
			Core.gemm(cameraRotationMatrix, rotationMatrix32F, 1, new Mat(), 0, MrcamXrotationMatrix, 0);

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
		Core.gemm(cameraRotationMatrix, MRotationFinale, 1, new Mat(), 0, cameraRotationMatrix, 0);

		Calib3d.Rodrigues(cameraRotationMatrix, Rcam);
		MatOfPoint3f zeroZeroZero = new MatOfPoint3f(new Point3(0, 0, 0));
		MatOfPoint2f centre = new MatOfPoint2f();
		Calib3d.projectPoints(zeroZeroZero, Rcam, translationVector, intrinsicParametersMatrix, distortionCoefficients, centre);

		Mat centerMatrix = new Mat(3, 1, CvType.CV_32F);
		centerMatrix.put(0, 0, centre.get(0, 0)[0]);
		centerMatrix.put(1, 0, centre.get(0, 0)[1]);
		centerMatrix.put(2, 0, 1);

		Mat Ntvec = new Mat(3, 1, CvType.CV_32F);
		Core.gemm(intrinsicParametersMatrix.inv(), centerMatrix, 1, new Mat(), 0, Ntvec, 0);
		double indice = Math.sqrt(cameraTranslationVector.get(0, 0)[0] * cameraTranslationVector.get(0, 0)[0] + cameraTranslationVector.get(1, 0)[0] * cameraTranslationVector.get(1, 0)[0] + cameraTranslationVector.get(2, 0)[0] * cameraTranslationVector.get(2, 0)[0]);
		Core.multiply(Ntvec, new Scalar(indice), Ntvec);
		Core.gemm(cameraRotationMatrix.inv(), Ntvec, 1, new Mat(), 0, cameraTranslationVector, 0);

		MatOfPoint2f VX = new MatOfPoint2f();
		Calib3d.projectPoints(new MatOfPoint3f(new Point3(0, 0, 0), new Point3(0.58, 0, 0)), Rcam, Ntvec, intrinsicParametersMatrix, distortionCoefficients, VX);
		MatOfPoint2f VY = new MatOfPoint2f();
		Calib3d.projectPoints(new MatOfPoint3f(new Point3(0, 0, 0), new Point3(0, 0.58, 0)), Rcam, Ntvec, intrinsicParametersMatrix, distortionCoefficients, VY);
		MatOfPoint2f VZ = new MatOfPoint2f();
		Calib3d.projectPoints(new MatOfPoint3f(new Point3(0, 0, 0), new Point3(0, 0, 0.58)), Rcam, Ntvec, intrinsicParametersMatrix, distortionCoefficients, VZ);

		Core.line(image, VX.toArray()[0], VX.toArray()[1], new Scalar(255, 255, 0), 2); // Android : Bleu // OpenCV : Bleu ciel
		Core.line(image, VY.toArray()[0], VY.toArray()[1], new Scalar(0, 255, 255), 2); // Android : Vert // OpenCV : Jaune
		Core.line(image, VZ.toArray()[0], VZ.toArray()[1], new Scalar(255, 0, 255), 2); // Android : Rouge // OpenCV : Rose

		MatOfPoint2f PointsCaptes = new MatOfPoint2f();
		Calib3d.projectPoints(objectPoints, rotationVector, translationVector, intrinsicParametersMatrix, distortionCoefficients, PointsCaptes);

		for (Point p : PointsCaptes.toArray()) {
			Core.circle(image, p, 20, new Scalar(255, 0, 0));
		}

		MatrixTransformations.MrcamX = cameraRotationMatrix;
		MatrixTransformations.cameraMatrix = intrinsicParametersMatrix;
		MatrixTransformations.cameraTranslationVector = cameraTranslationVector;
	}

	private static Mat matrixRotationFromX(Mat matrix, double angle) {

		// Matrice de rotation
		Mat rotationMatrixFromXAxis = new Mat(3, 3, CvType.CV_32F);
		rotationMatrixFromXAxis.put(0, 0, 1);
		rotationMatrixFromXAxis.put(0, 1, 0);
		rotationMatrixFromXAxis.put(0, 2, 0);
		rotationMatrixFromXAxis.put(1, 0, 0);
		rotationMatrixFromXAxis.put(1, 1, Math.cos(angle));
		rotationMatrixFromXAxis.put(1, 2, -Math.sin(angle));
		rotationMatrixFromXAxis.put(2, 0, 0);
		rotationMatrixFromXAxis.put(2, 1, Math.sin(angle));
		rotationMatrixFromXAxis.put(2, 2, Math.cos(angle));

		// La matrice matrix reçoit la multiplication de matrix par rotationMatrixFromXAxis
		Core.gemm(matrix, rotationMatrixFromXAxis, 1, new Mat(), 0, matrix, 0);

		return matrix;
	}

	private static float degreeToRadian(double degreeAngle) {
		return (float) (degreeAngle * Math.PI / 180.0);
	}

	private static Mat MrcamX;
	private static Mat cameraMatrix;
	private static Mat cameraTranslationVector;

	public static Point3 detect(Mat image, Point toDetect) throws Exception {

		if (image == null || toDetect == null)
			throw new Exception("VNC ERROR : Null parameter in detection.");

		if (MrcamX == null || cameraMatrix == null || cameraTranslationVector == null)
			throw new Exception("VNC ERROR : the matrix was not calculated before.");

		Mat touchedPointMatrix = new Mat(3, 1, CvType.CV_32F);
		touchedPointMatrix.put(0, 0, toDetect.x);
		touchedPointMatrix.put(1, 0, toDetect.y);
		touchedPointMatrix.put(2, 0, 1);

		Mat coordonneesClique = new Mat(3, 1, CvType.CV_32F);
		Core.gemm(cameraMatrix.inv(), touchedPointMatrix, 1, new Mat(), 0, coordonneesClique, 0);
		Core.gemm(MrcamX.inv(), coordonneesClique, 1, new Mat(), 0, coordonneesClique, 0);

		double t = -cameraTranslationVector.get(2, 0)[0] / coordonneesClique.get(2, 0)[0];
		double X = -coordonneesClique.get(0, 0)[0] * t - cameraTranslationVector.get(0, 0)[0];
		double Y = -coordonneesClique.get(1, 0)[0] * t - cameraTranslationVector.get(1, 0)[0];

		return new Point3(X, Y, 0);

	}

	protected static Mat buildIntrinsicParametersMatrix(Mat image) {

		// Récupération de la focale en pixels
		// TODO AMELIORATION : float focalLength = cameraParameters.getFocalLength();
		// float focalLengthInPixel = TypedValue.applyDimension(TypedValue.COMPLEX_UNIT_MM, CAMERA_FOCAL_LENGTH*1000, displayMetrics);

		// Coordonnées du centre de l'écran dans notre repère
		float centreX = (float) ((image.width() - 1) / 2);
		float centreY = (float) ((image.height() - 1) / 2);

		// Initialisation de la matrice des paramètres intrinsèques à la caméra et ajout des composants à cette matrice
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
}