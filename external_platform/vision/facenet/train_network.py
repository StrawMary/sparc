from aligndata_first import align_data
from create_classifier_se import create_new_classifier


def train_network(classifier_name="faces_classifier"):
	create_new_classifier(classifier_name)


if __name__ == '__main__':
	train_network()