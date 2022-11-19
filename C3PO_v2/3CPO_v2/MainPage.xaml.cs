using _3CPO_v2.ViewModel;

namespace _3CPO_v2;

public partial class MainPage : ContentPage
{
	public MainPage(MainViewModel vm)
	{
		InitializeComponent();
        BindingContext = vm;
	}

	
}

