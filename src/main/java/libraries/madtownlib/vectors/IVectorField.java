package libraries.madtownlib.vectors;

import libraries.cheesylib.geometry.Translation2d;

public interface IVectorField {
	public abstract Translation2d getVector(Translation2d here);
}