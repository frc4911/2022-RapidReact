package libraries.madtownlib.vectors;

import java.util.function.Function;

import libraries.cheesylib.geometry.Translation2d;

public abstract class Surface implements ISurface{

	public abstract Function<Translation2d, Double> f();

	public abstract Function<Translation2d, Double> dfdx();

	public abstract Function<Translation2d, Double> dfdy();
	
}
