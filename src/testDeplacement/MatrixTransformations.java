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

/**
 * Classe permettant d'effectuer divers calculs matriciels et utiles pour le calcul des coordonnées de la projection du point 2D cliqué à l'écran, dans le repère 3D du robot.
 * 
 * @author Maxime MARÇAIS.
 */
public class MatrixTransformations {

	private final static float CAMERA_FOCAL_LENGTH = 1; // 3.43f; // Exprimée en px
	private static Mat intrinsicParametersMatrix; // Matrice des paramètres intrinsèques à l'appareil
	private static Mat cameraTranslationVector; // Matrice des vecteurs de rotation de l'appareil
	private static Mat MrcamX;

	/**
	 * Construit une matrice constituée des paramètres intrinsèques à la caméra, soit la focale et le centre de l'écran de la caméra.
	 * 
	 * @param imageMatrix
	 *            la matrice des dimensions de l'écran de la caméra.
	 * @return la matrice des paramètres intrinsèques à la caméra.
	 */
	protected static Mat buildIntrinsicParametersMatrix(Mat imageMatrix) {

		// Récupération de la focale en pixels
		// TODO AMELIORATION : float focalLength = cameraParameters.getFocalLength();
		// float focalLengthInPixel = TypedValue.applyDimension(TypedValue.COMPLEX_UNIT_MM, CAMERA_FOCAL_LENGTH*1000, displayMetrics);

		// Coordonnées du centre de l'écran dans notre repère
		final float centreX = (imageMatrix.width() - 1) / 2;
		final float centreY = (imageMatrix.height() - 1) / 2;

		// Initialisation de la matrice des paramètres intrinsèques à la caméra et ajout des composants à cette matrice
		final Mat intrinsicParametersMatrix = new Mat(3, 3, CvType.CV_32F);
		intrinsicParametersMatrix.put(0, 0, MatrixTransformations.CAMERA_FOCAL_LENGTH * imageMatrix.width());
		intrinsicParametersMatrix.put(0, 1, 0);
		intrinsicParametersMatrix.put(0, 2, centreX);
		intrinsicParametersMatrix.put(1, 0, 0);
		intrinsicParametersMatrix.put(1, 1, MatrixTransformations.CAMERA_FOCAL_LENGTH * imageMatrix.width());
		intrinsicParametersMatrix.put(1, 2, centreY);
		intrinsicParametersMatrix.put(2, 0, 0);
		intrinsicParametersMatrix.put(2, 1, 0);
		intrinsicParametersMatrix.put(2, 2, 1);

		return intrinsicParametersMatrix;

	}

	/**
	 * Calcul les trois matrices nécessaires aux calculs pour le déplacement : intrinsicParametersMatrix, cameraTranslationVector et MrcamX
	 * 
	 * @param imageMatrix
	 *            la matrice de l'image
	 * @param roll
	 *            la rotation de l'appareil selon roll
	 * @param pitch
	 *            la rotation de l'appareil selon pitch
	 * @param azimuth
	 *            la rotation de l'appareil selon azimuth, non-utilisé ici
	 * @param objectPoints
	 *            la matrice des points choisis dans le repère 3D du robot
	 * @param imagePoints
	 *            la matrice de ces mêmes points mais dans le repère 2D correspondant, de la caméra
	 * @throws Exception
	 */
	public static void calcMatrix(Mat imageMatrix, float roll, float pitch, float azimuth, MatOfPoint3f objectPoints, MatOfPoint2f imagePoints) throws Exception {

		// Détection d'éventuelles données importantes manquantes
		if (imageMatrix == null || objectPoints == null || imagePoints == null)
			throw new Exception("VNC ERROR : Null parameter in detection."); // TODO : Remplacer cette Exception par une exception propre au projet et à cette erreur
		if (objectPoints.rows() != imagePoints.rows())
			throw new Exception("VNC ERROR : number of object points and image points are different."); // TODO : Remplacer cette Exception par une exception propre au projet et à cette erreur

		// Récupération des paramètres intrinsèques à la caméra
		final Mat intrinsicParametersMatrix = MatrixTransformations.buildIntrinsicParametersMatrix(imageMatrix);

		// Construction d'une matrices représentant les coefficients de distorsion de la caméra (aucune distorsion : matrice de zéros)
		final MatOfDouble distortionCoefficients = new MatOfDouble(new Mat(1, 4, CvType.CV_64F));

		// Récupération de la matrice de correspondance 2D/3D
		final Mat rotationVector = new Mat(); // Output de solvePnP
		final Mat translationVector = new Mat(); // Output de solvePnP
		Calib3d.solvePnP(objectPoints, imagePoints, intrinsicParametersMatrix, distortionCoefficients, rotationVector, translationVector);

		// Conversion vecteur de rotation vers matrice de rotation
		final Mat rotationMatrix = new Mat(); // Output de Rodrigues
		Calib3d.Rodrigues(rotationVector, rotationMatrix);

		// Construction de la matrice de passage
		final Mat cameraTranslationVector = new Mat(3, 1, CvType.CV_32F);
		Core.gemm(rotationMatrix.inv(), translationVector, 1, new Mat(), 0, cameraTranslationVector, 0);

		// Construction de la matrice de rotation qui correspond à la rotation que fait l'appareil autours de son axe x
		roll = MatrixTransformations.degreeToRadian(roll);
		final Mat cameraRotationMatrixAroundX = new Mat(3, 3, CvType.CV_32F);
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
		pitch = MatrixTransformations.degreeToRadian(pitch);
		final Mat cameraRotationMatrixAroundY = new Mat(3, 3, CvType.CV_32F);
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

		// Rotation du repère appareil dans celui de la caméra openCV (-PI/2 selon l'axe x)
		cameraRotationMatrix = MatrixTransformations.matrixRotationAroundX(cameraRotationMatrix, Math.PI);

		final Mat Rcam = new Mat(); // Output de Rodrigues
		Calib3d.Rodrigues(cameraRotationMatrix, Rcam);

		final Mat rotationMatrix32F = new Mat(3, 3, CvType.CV_32F);
		rotationMatrix32F.put(0, 0, rotationMatrix.get(0, 0)[0]);
		rotationMatrix32F.put(0, 1, rotationMatrix.get(0, 1)[0]);
		rotationMatrix32F.put(0, 2, rotationMatrix.get(0, 2)[0]);
		rotationMatrix32F.put(1, 0, rotationMatrix.get(1, 0)[0]);
		rotationMatrix32F.put(1, 1, rotationMatrix.get(1, 1)[0]);
		rotationMatrix32F.put(1, 2, rotationMatrix.get(1, 2)[0]);
		rotationMatrix32F.put(2, 0, rotationMatrix.get(2, 0)[0]);
		rotationMatrix32F.put(2, 1, rotationMatrix.get(2, 1)[0]);
		rotationMatrix32F.put(2, 2, rotationMatrix.get(2, 2)[0]);

		final Mat MrcamRotationX = new Mat(3, 3, CvType.CV_32F);
		Core.gemm(cameraRotationMatrix, rotationMatrix32F, 1, new Mat(), 0, MrcamRotationX, 0);

		final Mat Xrob = new Mat(3, 1, CvType.CV_32F);
		final Mat oneZeroZeroMatrix = new Mat(3, 1, CvType.CV_32F);
		oneZeroZeroMatrix.put(0, 0, 1);
		oneZeroZeroMatrix.put(1, 0, 0);
		oneZeroZeroMatrix.put(2, 0, 0);
		Core.gemm(MrcamRotationX, oneZeroZeroMatrix, 1, new Mat(), 0, Xrob, 0);

		final Mat MRotation = new Mat(3, 3, CvType.CV_32F);

		double resMax = 0;
		final Mat MRotationFinale = new Mat(3, 3, CvType.CV_32F);
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

			final Mat Rrotation = new Mat(); // Output de Rodrigues
			Calib3d.Rodrigues(MRotation, Rrotation);

			final Mat MrcamXrotationMatrix = new Mat(3, 3, CvType.CV_32F);
			Core.gemm(cameraRotationMatrix, rotationMatrix32F, 1, new Mat(), 0, MrcamXrotationMatrix, 0);

			final Mat MRotationMrcamXrotationMatrix = new Mat(3, 3, CvType.CV_32F);
			Core.gemm(MRotation, MrcamXrotationMatrix, 1, new Mat(), 0, MRotationMrcamXrotationMatrix, 0);

			Core.gemm(MRotationMrcamXrotationMatrix, oneZeroZeroMatrix, 1, new Mat(), 0, Xrob, 0);

			final double res = Xrob.get(0, 0)[0];

			if (res > resMax) {
				resMax = res;
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
		final MatOfPoint3f zeroZeroZero = new MatOfPoint3f(new Point3(0, 0, 0));
		final MatOfPoint2f centre = new MatOfPoint2f();
		Calib3d.projectPoints(zeroZeroZero, Rcam, translationVector, intrinsicParametersMatrix, distortionCoefficients, centre);

		final Mat centerMatrix = new Mat(3, 1, CvType.CV_32F);
		centerMatrix.put(0, 0, centre.get(0, 0)[0]);
		centerMatrix.put(1, 0, centre.get(0, 0)[1]);
		centerMatrix.put(2, 0, 1);

		final Mat Ntvec = new Mat(3, 1, CvType.CV_32F);
		Core.gemm(intrinsicParametersMatrix.inv(), centerMatrix, 1, new Mat(), 0, Ntvec, 0);
		final double indice = Math.sqrt(cameraTranslationVector.get(0, 0)[0] * cameraTranslationVector.get(0, 0)[0] + cameraTranslationVector.get(1, 0)[0] * cameraTranslationVector.get(1, 0)[0] + cameraTranslationVector.get(2, 0)[0] * cameraTranslationVector.get(2, 0)[0]);
		Core.multiply(Ntvec, new Scalar(indice), Ntvec);
		Core.gemm(cameraRotationMatrix.inv(), Ntvec, 1, new Mat(), 0, cameraTranslationVector, 0);

		final MatOfPoint2f VX = new MatOfPoint2f();
		Calib3d.projectPoints(new MatOfPoint3f(new Point3(0, 0, 0), new Point3(0.58, 0, 0)), Rcam, Ntvec, intrinsicParametersMatrix, distortionCoefficients, VX);
		final MatOfPoint2f VY = new MatOfPoint2f();
		Calib3d.projectPoints(new MatOfPoint3f(new Point3(0, 0, 0), new Point3(0, 0.58, 0)), Rcam, Ntvec, intrinsicParametersMatrix, distortionCoefficients, VY);
		final MatOfPoint2f VZ = new MatOfPoint2f();
		Calib3d.projectPoints(new MatOfPoint3f(new Point3(0, 0, 0), new Point3(0, 0, 0.58)), Rcam, Ntvec, intrinsicParametersMatrix, distortionCoefficients, VZ);

		Core.line(imageMatrix, VX.toArray()[0], VX.toArray()[1], new Scalar(255, 255, 0), 2); // Android : Bleu // OpenCV : Bleu ciel
		Core.line(imageMatrix, VY.toArray()[0], VY.toArray()[1], new Scalar(0, 255, 255), 2); // Android : Vert // OpenCV : Jaune
		Core.line(imageMatrix, VZ.toArray()[0], VZ.toArray()[1], new Scalar(255, 0, 255), 2); // Android : Rouge // OpenCV : Rose

		final MatOfPoint2f PointsCaptes = new MatOfPoint2f();
		Calib3d.projectPoints(objectPoints, rotationVector, translationVector, intrinsicParametersMatrix, distortionCoefficients, PointsCaptes);

		for (final Point p : PointsCaptes.toArray())
			Core.circle(imageMatrix, p, 20, new Scalar(255, 0, 0));

		MatrixTransformations.MrcamX = cameraRotationMatrix;
		MatrixTransformations.intrinsicParametersMatrix = intrinsicParametersMatrix;
		MatrixTransformations.cameraTranslationVector = cameraTranslationVector;
	}

	/**
	 * Transforme un angle en degré en un angle en radian.
	 * 
	 * @param degreeAngle
	 *            l'angle en degré.
	 * @return la valeur de l'angle en radian.
	 */
	private static float degreeToRadian(double degreeAngle) {
		return (float) (degreeAngle * Math.PI / 180.0);
	}

	/**
	 * Détection d'un point 2D, dans le repère 3D du robot
	 * 
	 * @param pointToDetect
	 *            le point 2D à détecter dans le repère 3D
	 * @return le point 3D correspondant à cette détection
	 * @throws Exception
	 */
	public static Point3 detect(Point pointToDetect) throws Exception {

		if (MatrixTransformations.MrcamX == null || MatrixTransformations.intrinsicParametersMatrix == null || MatrixTransformations.cameraTranslationVector == null)
			throw new Exception("VNC ERROR : the matrix was not calculated before."); // TODO : Remplacer cette Exception par une exception propre au projet et à cette erreur

		// Initialisation de la matrice de la position du point 2D sélectionné
		final Mat touchedPointMatrix = new Mat(3, 1, CvType.CV_32F);
		touchedPointMatrix.put(0, 0, pointToDetect.x);
		touchedPointMatrix.put(1, 0, pointToDetect.y);
		touchedPointMatrix.put(2, 0, 1);

		// Calcul des coordonnées du point cliqué dans le le repère 3D du robot
		final Mat clickedPointCoordinates = new Mat(3, 1, CvType.CV_32F);
		Core.gemm(MatrixTransformations.intrinsicParametersMatrix.inv(), touchedPointMatrix, 1, new Mat(), 0, clickedPointCoordinates, 0);
		Core.gemm(MatrixTransformations.MrcamX.inv(), clickedPointCoordinates, 1, new Mat(), 0, clickedPointCoordinates, 0);

		final double t = -MatrixTransformations.cameraTranslationVector.get(2, 0)[0] / clickedPointCoordinates.get(2, 0)[0];
		final double x = -clickedPointCoordinates.get(0, 0)[0] * t - MatrixTransformations.cameraTranslationVector.get(0, 0)[0];
		final double y = -clickedPointCoordinates.get(1, 0)[0] * t - MatrixTransformations.cameraTranslationVector.get(1, 0)[0];

		return new Point3(x, y, 0);

	}

	/**
	 * Affiche dans la console la représentation (sous forme de matrice) de la matrice matrix.
	 * 
	 * @param matrix
	 *            matrice à afficher.
	 * @param name
	 *            nom de cette dernière, en réalité ceci est juste un affichage et n'a aucune incidence.
	 */
	public static void displayMatrix(Mat matrix, String name) {
		System.out.println("MATRIX : " + name);
		String line = "";
		for (int i = 0; i < matrix.rows(); i++) {
			for (int j = 0; j < matrix.cols(); j++)
				line += matrix.get(i, j)[0] + " ";
			System.out.println("MATRIX : " + line);
			line = "";
		}
	}

	/**
	 * Fait la rotation d'une matrice autours de l'axe x selon un angle angle.
	 * 
	 * @param matrix
	 *            la matrice à transformer.
	 * @param angle
	 *            l'angle de la rotation autours de l'axe x.
	 * @return la matrice transformée.
	 */
	private static Mat matrixRotationAroundX(Mat matrix, double angle) {

		// Matrice de rotation
		final Mat rotationMatrixAroundXAxis = new Mat(3, 3, CvType.CV_32F);
		rotationMatrixAroundXAxis.put(0, 0, 1);
		rotationMatrixAroundXAxis.put(0, 1, 0);
		rotationMatrixAroundXAxis.put(0, 2, 0);
		rotationMatrixAroundXAxis.put(1, 0, 0);
		rotationMatrixAroundXAxis.put(1, 1, Math.cos(angle));
		rotationMatrixAroundXAxis.put(1, 2, -Math.sin(angle));
		rotationMatrixAroundXAxis.put(2, 0, 0);
		rotationMatrixAroundXAxis.put(2, 1, Math.sin(angle));
		rotationMatrixAroundXAxis.put(2, 2, Math.cos(angle));

		// La matrice matrix reçoit la multiplication de matrix par rotationMatrixFromXAxis
		Core.gemm(matrix, rotationMatrixAroundXAxis, 1, new Mat(), 0, matrix, 0);

		return matrix;
	}

}