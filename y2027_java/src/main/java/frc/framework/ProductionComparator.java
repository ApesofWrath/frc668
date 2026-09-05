package frc.framework;

import java.util.Comparator;

/**
 * Internal comparator that allows productions to be sorted by priority.
 */
public class ProductionComparator implements Comparator<Production> {

	@Override
	public int compare(Production o1, Production o2) {
		return -Integer.valueOf(o1.priority.value).compareTo(Integer.valueOf(o2.priority.value));
	}
    
}
