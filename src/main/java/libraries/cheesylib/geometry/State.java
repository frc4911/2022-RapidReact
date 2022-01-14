package libraries.cheesylib.geometry;

import libraries.cheesylib.util.CSVWritable;
import libraries.cheesylib.util.Interpolable;

public interface State<S> extends Interpolable<S>, CSVWritable {
    double distance(final S other);

    boolean equals(final Object other);

    String toString();

    String toCSV();
}
